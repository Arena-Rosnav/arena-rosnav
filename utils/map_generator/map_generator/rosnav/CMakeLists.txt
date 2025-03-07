cmake_minimum_required(VERSION 3.5)
project(rosnav)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



find_package(ament_cmake REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES} ${geometry_msgs_LIBRARIES} ${rclpy_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${tf2_ros_LIBRARIES}
  ${nav_msgs_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES} ${geometry_msgs_LIBRARIES} ${rclpy_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${tf2_ros_LIBRARIES}
  ${nav_msgs_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ResetStackedObs.msg"
  "srv/GetAction.srv"
  DEPENDENCIES builtin_interfaces tf2_geometry_msgs geometry_msgs nav_msgs)






install(PROGRAMS scripts/rosnav_node.py
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(nav_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
