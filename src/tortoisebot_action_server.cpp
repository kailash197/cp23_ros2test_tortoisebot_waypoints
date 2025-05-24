#include "tortoisebot_waypoints/action/waypoint.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.h>

using namespace std::placeholders;
using Waypoint = tortoisebot_waypoints::action::Waypoint;
using GoalHandle = rclcpp_action::ServerGoalHandle<Waypoint>;

class TortoisebotActionServer : public rclcpp::Node {
public:
  explicit TortoisebotActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point current_position_;
  double current_yaw_;
  std::string state_;

  const double yaw_precision_ = (8 * M_PI) / 180;
  const double dist_precision_ = 0.1;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double get_yaw_from_quat(const geometry_msgs::msg::Quaternion &quat);
  double normalize_angle(double angle);
  double calculate_desired_yaw(const geometry_msgs::msg::Point &goal_position);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

TortoisebotActionServer::TortoisebotActionServer(
    const rclcpp::NodeOptions &options)
    : Node("waypoint_action_server", options) {
  action_server_ = rclcpp_action::create_server<Waypoint>(
      this, "tortoisebot_as",
      std::bind(&TortoisebotActionServer::handle_goal, this, _1, _2),
      std::bind(&TortoisebotActionServer::handle_cancel, this, _1),
      std::bind(&TortoisebotActionServer::handle_accepted, this, _1));

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TortoisebotActionServer::odom_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Waypoint Action Server started");
}

void TortoisebotActionServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_position_ = msg->pose.pose.position;
  current_yaw_ = get_yaw_from_quat(msg->pose.pose.orientation);
  current_yaw_ = normalize_angle(current_yaw_); // Normalize current yaw
}

double TortoisebotActionServer::get_yaw_from_quat(
    const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  return tf2::getYaw(tf_quat);
}

double TortoisebotActionServer::normalize_angle(double angle) {
  // Normalize angle to [-π, π]
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double TortoisebotActionServer::calculate_desired_yaw(
    const geometry_msgs::msg::Point &goal_position) {
  double dx = goal_position.x - current_position_.x;
  double dy = goal_position.y - current_position_.y;
  return normalize_angle(atan2(dy, dx));
}

rclcpp_action::GoalResponse TortoisebotActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Waypoint::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TortoisebotActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TortoisebotActionServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  std::thread{std::bind(&TortoisebotActionServer::execute, this, _1),
              goal_handle}
      .detach();
}

void TortoisebotActionServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle) {
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Waypoint::Feedback>();
  auto result = std::make_shared<Waypoint::Result>();

  rclcpp::Rate loop_rate(25);
  bool success = true;

  while (rclcpp::ok() && success) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    double desired_yaw = calculate_desired_yaw(goal->position);
    double err_yaw = normalize_angle(desired_yaw - current_yaw_);
    double err_pos = sqrt(pow(goal->position.y - current_position_.y, 2) +
                          pow(goal->position.x - current_position_.x, 2));

    auto twist_msg = geometry_msgs::msg::Twist();

    if (fabs(err_yaw) > yaw_precision_) {
      state_ = "fix yaw";
      // Choose the shortest rotation direction
      twist_msg.angular.z = (err_yaw > 0) ? 0.5 : -0.5;
    } else if (err_pos > dist_precision_) {
      state_ = "go to point";
      twist_msg.linear.x = 0.45;
    } else {
      break;
    }

    cmd_vel_pub_->publish(twist_msg);

    feedback->position = current_position_;
    feedback->state = state_;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // Stop the robot
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());

  if (success) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TortoisebotActionServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
