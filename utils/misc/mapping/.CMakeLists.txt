cmake_minimum_required(VERSION 3.5)
project(mapping)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}  
  ${visualization_msgs_INCLUDE_DIRS}
)




## Declare a C++ library
add_library(mapping
  src/mapping.cpp
  src/raycast.cpp
)

## Specify libraries to link against
target_link_libraries(mapping
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

## Install
install(
  TARGETS mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

## Export dependencies
ament_export_dependencies(
  ament_cmake
  message_filters
  rclpy
  std_msgs
  std_srvs
  pcl_conversions
  sensor_msgs
  visualization_msgs
  tf2
  pcl_ros
  Eigen3
  PCL
  rclcpp
  laser_geometry
  nav_msgs
)

ament_package()
