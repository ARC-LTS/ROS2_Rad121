cmake_minimum_required(VERSION 3.8)
project(rad_local_recovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_rad_map REQUIRED)
find_package(action_msgs REQUIRED)

add_executable(local_recovery_node src/local_recovery_node.cpp)
ament_target_dependencies(local_recovery_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
  ros2_rad_map
)

# General-purpose, radiation-independent stuck-detection watchdog: monitors
# actual odometry movement (not just goal status) while a navigate_to_pose
# goal is executing, and physically backs the robot away if it hasn't
# moved for stuck_timeout_s. Was written but never added to the build or
# any launch file -- previously the only things that could get the robot
# out of a geometric dead end (a corner, independent of radiation) were
# Nav2's own recovery escalation (rarely triggered -- observed 5+ silent
# "Failed to make progress" retries before a single real recovery fired)
# and explore_lite's frontier replanning (which only picks a new target;
# it doesn't physically move the robot out of a spot no path can leave).
add_executable(stuck_escape_node src/stuck_escape_node.cpp)
ament_target_dependencies(stuck_escape_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  action_msgs
)

install(TARGETS
  local_recovery_node
  stuck_escape_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_package()
