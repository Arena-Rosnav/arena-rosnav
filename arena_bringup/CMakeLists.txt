cmake_minimum_required(VERSION 3.5)
project(arena_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}/
)

# add scripts
install(
  PROGRAMS
  scripts/pull
  scripts/repos
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()