cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

include_directories(include)

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server
  rclcpp rclcpp_action geometry_msgs tf2 tf2_ros nav_msgs
)

rosidl_target_interfaces(tortoisebot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  tortoisebot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

add_library(${PROJECT_NAME}_lib src/tortoisebot_action_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

rosidl_target_interfaces(${PROJECT_NAME}_lib
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/main.cpp
    test/tortoisebot_action_server_test.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
	rclcpp
	rclcpp_action
	geometry_msgs
	nav_msgs
	std_srvs
	tf2
	tf2_ros
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_lib
  )

  install(TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
