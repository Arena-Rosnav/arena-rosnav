cmake_minimum_required(VERSION 3.5)
project(task_generator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS 
  scripts/main.py 
  scripts/scenario_helper.py
  scripts/server.py 
  scripts/filewatcher.py 
  scripts/delay_restart.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()



