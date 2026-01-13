cmake_minimum_required(VERSION 3.8)
project(rad121_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBFTDI REQUIRED libftdi1)

include_directories(include ${LIBFTDI_INCLUDE_DIRS})

add_executable(rad121_monitor_node src/rad121_monitor_node.cpp src/CUSB_RAD121.cpp)
ament_target_dependencies(rad121_monitor_node rclcpp std_msgs)
target_link_libraries(rad121_monitor_node ${LIBFTDI_LIBRARIES})

install(TARGETS
  rad121_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)



install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_package()
