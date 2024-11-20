cmake_minimum_required(VERSION 3.5)
project(plan_visualization)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(plan_visualization src/planning_visualization.cpp)

# Link libraries
target_link_libraries(plan_visualization
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  Eigen3::Eigen
  ${nav_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)

# Install targets
install(
  TARGETS plan_visualization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(
  ament_cmake
  std_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  Eigen3
  nav_msgs
  visualization_msgs
)

ament_export_include_directories(include)
ament_export_libraries(plan_visualization)

ament_package()
