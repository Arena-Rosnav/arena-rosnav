cmake_minimum_required(VERSION 3.5)
project(map_generator)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(map_distance_server REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${map_distance_server_INCLUDE_DIRS} ${rosidl_default_generators_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${std_msgs_LIBRARIES} ${map_distance_server_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${map_distance_server_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})


install(PROGRAMS map_generator/map_generator_node.py
  map_generator/map_server.py
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(map_distance_server)
ament_export_dependencies(rosidl_default_generators)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
