cmake_minimum_required(VERSION 3.5)
project(arena_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Uncomment if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# ament_python_install_package(${PROJECT_NAME})

# Uncomment this section to add messages, services, or actions
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Message1.msg"
#   "srv/Service1.srv"
# )

# Uncomment and add dynamic reconfigure files as necessary
# find_package(dynamic_reconfigure REQUIRED)
# generate_dynamic_reconfigure_options(cfg/DynReconf1.cfg)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/arena_bringup.cpp
# )

# Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/arena_bringup_node.cpp)

# Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable/library
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

# Add libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${ament_LIBRARIES}
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Mark executables for installation
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
# )

# Mark other files for installation
# install(FILES
#   launch/my_launch_file.launch.py
#   DESTINATION share/${PROJECT_NAME}
# )

# ament package
ament_package()

# Uncomment this if testing is required
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

