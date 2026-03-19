cmake_minimum_required(VERSION 3.8)
project(rad_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/RadiationLayer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav2_costmap_2d
  sensor_msgs
  cv_bridge
  nav_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
