cmake_minimum_required(VERSION 3.5)
project(grid_map_generator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)


install(PROGRAMS
  scripts/grid_map_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()

