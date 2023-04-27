cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
  message_generation
  map_distance_server
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime nav_msgs
)

catkin_install_python(PROGRAMS
  map_generator/map_generator_node.py
  map_generator/map_server.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
