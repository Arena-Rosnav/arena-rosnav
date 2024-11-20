cmake_minimum_required(VERSION 3.5)
project(plan_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/RobotState.msg"
  "msg/RobotStateStamped.msg"
)

set(srv_files
  "srv/Subgoal.srv"
  "srv/MakeGlobalPlan.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Uncomment the following lines if you have C++ code to build
# include_directories(include)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/plan_msgs.cpp
# )
# target_link_libraries(${PROJECT_NAME}
#   ${rosidl_default_runtime_LIBRARIES}
# )

# Add executable
# add_executable(${PROJECT_NAME}_node src/plan_msgs_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
#   ${rosidl_default_runtime_LIBRARIES}
# )

# Install targets
# install(TARGETS
#   ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
