cmake_minimum_required(VERSION 3.5)
project(observations)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Uncomment if needed
# find_package(SDL2 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(OpenCV REQUIRED)

# Generate messages - do this early
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Observation.msg"
  DEPENDENCIES
    sensor_msgs
    geometry_msgs
    std_msgs
)

# Create observation_packer executable
add_executable(observation_packer
  observation_packer/observation_packer.cpp
  common/zVector2d.cpp
  common/zVector2d.h
  common/f_math.cpp
  common/f_math.h
)

# Add dependencies
ament_target_dependencies(observation_packer
  rclcpp
  rcutils
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  geometry_msgs
)

# Make sure our messages are built before the executable
add_dependencies(observation_packer
  ${PROJECT_NAME}
)

# Include directories
target_include_directories(observation_packer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ./
  # Uncomment if needed
  # ${SDL2_INCLUDE_DIRS}
  # ${OPENGL_INCLUDE_DIRS}
)

# Add message interface and link against it
rosidl_target_interfaces(observation_packer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Additional libraries (if needed)
target_link_libraries(observation_packer
  # ${SDL2_LIBRARIES}
  # ${OPENGL_LIBRARIES}
)

# Install targets
install(TARGETS
  observation_packer
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (if any)
# install(PROGRAMS
#   agents/agent_play.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install header files
install(DIRECTORY
  common/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rcutils
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Export include directories
ament_export_include_directories(
  include  # This is for your package's headers
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # This is for generated message headers
)

# Finish package
ament_package()