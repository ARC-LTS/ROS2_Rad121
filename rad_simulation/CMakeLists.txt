cmake_minimum_required(VERSION 3.8)
project(rad_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat64.srv"
  "srv/AddSource.srv"
  "srv/SpawnSource.srv"
  "srv/SpawnRandomSources.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(rad_field_server src/rad_field_server.cpp)
ament_target_dependencies(rad_field_server
  rclcpp
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(rad_field_server yaml-cpp "${cpp_typesupport_target}")

find_package(yaml-cpp REQUIRED)

add_executable(source_selector_node src/source_selector_node.cpp)
ament_target_dependencies(source_selector_node
  rclcpp
  geometry_msgs
  std_srvs
)
target_link_libraries(source_selector_node yaml-cpp "${cpp_typesupport_target}")

add_executable(rad_sensor_simulator src/rad_sensor_simulator.cpp)
ament_target_dependencies(rad_sensor_simulator
  rclcpp
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  rad_field_server
  rad_sensor_simulator
  source_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
