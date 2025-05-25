#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint.hpp"
#include "gtest/gtest.h"
#include <memory>

#define POSITION_TOLERANCE 0.1
#define YAW_TOLERANCE M_PI / 12 // 15 degrees

using namespace std::chrono_literals;
using Waypoint = tortoisebot_waypoints::action::Waypoint;
using GoalHandle = rclcpp_action::ClientGoalHandle<Waypoint>;
using Empty = std_srvs::srv::Empty;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class TortoisebotActionServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("tortoisebot_as_test_node");

    // Initialize action client
    tortoisebot_as_client_ =
        rclcpp_action::create_client<Waypoint>(node_, "tortoisebot_as");

    // Initialize odometry subscriber
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_position_ = msg->pose.pose.position;
          current_orientation_ = msg->pose.pose.orientation;
          odom_received_ = true;
        });

    // Initialize cmd_vel publisher
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize reset service client
    reset_client_ = node_->create_client<Empty>("/reset_world");

    // Get test parameter
    const char *test_env = std::getenv("TEST_PASS");
    test_pass_ = (test_env && std::string(test_env) == "false") ? false : true;

    // node_->declare_parameter("test_pass", true);
    // test_pass_ = node_->get_parameter("test_pass").as_bool();

    // Set test goals
    if (test_pass_) {
      goal_.position.x = 1.0;
      goal_.position.y = 1.0;
      expected_yaw_ = M_PI / 4.0; // 45 degrees
    } else {
      goal_.position.x = -2.0;
      goal_.position.y = -2.0;
      expected_yaw_ = -M_PI / 4.0; // -45 degrees
    }
  }

  void TearDown() override {
    stop_robot();
    node_.reset();
  }

  // Getter methods
  bool get_result_received() const { return result_received_; }
  const Waypoint::Goal &get_goal() const { return goal_; }
  const geometry_msgs::msg::Point &get_current_position() const {
    return current_position_;
  }
  double get_final_yaw() const { return final_yaw_; }
  double get_expected_yaw() const { return expected_yaw_; }

  void RunOrientationTest() {
    reset_simulation();

    // Wait for first odometry message
    while (!odom_received_) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(100ms);
    }

    initial_yaw_ = get_current_yaw();
    send_goal();

    // Wait for result with timeout
    auto start_time = rclcpp::Clock().now();
    while (!result_received_ &&
           (rclcpp::Clock().now() - start_time) < rclcpp::Duration(20s)) {
      rclcpp::spin_some(node_);
    }
    stop_robot();
  }

  void RunPositionTest() {
    reset_simulation();

    // Wait for first odometry message
    while (!odom_received_) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(100ms);
    }

    send_goal();

    // Wait for result with timeout
    auto start_time = rclcpp::Clock().now();
    while (!result_received_ &&
           (rclcpp::Clock().now() - start_time) < rclcpp::Duration(20s)) {
      rclcpp::spin_some(node_);
    }
    stop_robot();
  }

private:
  std::shared_ptr<rclcpp::Node> node_;

  // ROS 2 components
  rclcpp_action::Client<Waypoint>::SharedPtr tortoisebot_as_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<Empty>::SharedPtr reset_client_;

  // Test state
  bool test_pass_;
  bool result_received_ = false;
  bool odom_received_ = false;
  Waypoint::Goal goal_;
  double expected_yaw_;
  double initial_yaw_;
  double final_yaw_;

  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Quaternion current_orientation_;

  // Helper methods
  double get_current_yaw() {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(current_orientation_, tf_quat);
    return tf2::getYaw(tf_quat);
  }

  void reset_simulation() {
    while (!reset_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for reset service...");
    }

    auto request = std::make_shared<Empty::Request>();
    auto future = reset_client_->async_send_request(request);
  }

  void stop_robot() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
  }

  void send_goal() {
    if (!tortoisebot_as_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available");
      return;
    }

    auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](std::shared_future<GoalHandle::SharedPtr> future) {
          auto goal_handle = future.get();
          if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
          }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const Waypoint::Feedback>) {
          // Feedback handling if needed
        };

    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
          result_received_ = true;
          final_yaw_ = get_current_yaw();
        };

    auto future_goal_handle =
        tortoisebot_as_client_->async_send_goal(goal_, send_goal_options);
  }
};

TEST_F(TortoisebotActionServerTest, TestPosition) {
  RunPositionTest();
  ASSERT_TRUE(get_result_received());
  EXPECT_NEAR(get_goal().position.x, get_current_position().x,
              POSITION_TOLERANCE);
  EXPECT_NEAR(get_goal().position.y, get_current_position().y,
              POSITION_TOLERANCE);
}

TEST_F(TortoisebotActionServerTest, TestOrientation) {
  RunOrientationTest();
  ASSERT_TRUE(get_result_received());
  EXPECT_NEAR(get_final_yaw(), get_expected_yaw(), YAW_TOLERANCE);
}
