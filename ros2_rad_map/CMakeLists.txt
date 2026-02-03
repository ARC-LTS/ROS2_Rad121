cmake_minimum_required(VERSION 3.8)
project(ros2_rad_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS Common IO)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GPPredict.srv"
  "srv/FindPeaks.srv"
  "srv/GetHeatmaps.srv"
  "srv/SaveMaps.srv"
  "msg/RadZone.msg"
  "msg/RadZoneArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(rad_heatmap_node src/rad_heatmap_node.cpp)
ament_target_dependencies(rad_heatmap_node
  rclcpp
  cv_bridge
  sensor_msgs
  image_transport
  std_msgs
  std_srvs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(rad_heatmap_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(rad_pointcloud_node src/rad_pointcloud_node.cpp)
ament_target_dependencies(rad_pointcloud_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  pcl_conversions
  octomap_msgs
)
target_link_libraries(rad_pointcloud_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(rad_mapper_node src/rad_mapper_node.cpp)
ament_target_dependencies(rad_mapper_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  std_srvs
  nav_msgs
  cv_bridge
  visualization_msgs
)

add_executable(peak_finder_client_node src/peak_finder_client_node.cpp)
ament_target_dependencies(peak_finder_client_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(peak_finder_client_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(gp_visualizer_node src/gp_visualizer_node.cpp)
ament_target_dependencies(gp_visualizer_node
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gp_visualizer_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(rad_costmap_visualizer_node src/rad_costmap_visualizer_node.cpp)
ament_target_dependencies(rad_costmap_visualizer_node
  rclcpp
  cv_bridge
  sensor_msgs
  nav_msgs
  visualization_msgs
)

install(TARGETS
  rad_heatmap_node
  rad_mapper_node
  rad_pointcloud_node
  peak_finder_client_node
  gp_visualizer_node
  rad_costmap_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gp_predictor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/)

install (DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_package()
