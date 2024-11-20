cmake_minimum_required(VERSION 3.5)
project(eval)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

# Install launch files
# ament_package()

install(PROGRAMS
  src/run_evals.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
