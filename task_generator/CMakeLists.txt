cmake_minimum_required(VERSION 2.8.3)
project(task_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TaskGenerator.cfg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/scenario_helper.py
  scripts/server.py
  scripts/filewatcher.py
  scripts/delay_restart.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
