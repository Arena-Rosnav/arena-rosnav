cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs 
  geometry_msgs
  rospy
  message_generation
  map_distance_server
)

add_message_files(
  FILES
  Obstacle.msg
  MapObstacles.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime nav_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  map_generator/map_generator_node.py
  map_generator/map_server.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
