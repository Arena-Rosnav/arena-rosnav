cmake_minimum_required(VERSION 3.5)
project(sensor_simulator)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
)

# Uncomment and adjust the following lines to add your source files
# add_library(${PROJECT_NAME} src/sensor_simulator.cpp)

# Uncomment and adjust the following lines to add your executables
# add_executable(${PROJECT_NAME}_node src/sensor_simulator_node.cpp)

# If you uncomment the above, make sure to link the necessary libraries
# target_link_libraries(${PROJECT_NAME}_node
#   ${rclcpp_LIBRARIES}
#   ${std_msgs_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${nav_msgs_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
#   ${pcl_msgs_LIBRARIES}
# )

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#)

#install(TARGETS ${PROJECT_NAME}_node
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(rclcpp)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
