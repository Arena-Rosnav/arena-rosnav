cmake_minimum_required(VERSION 3.0.2)
project(map_distance_server)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GetDistanceMap.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime nav_msgs
)

catkin_install_python(PROGRAMS
  scripts/map_distance_node.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
