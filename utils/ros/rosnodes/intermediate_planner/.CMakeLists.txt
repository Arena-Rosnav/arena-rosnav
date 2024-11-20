cmake_minimum_required(VERSION 3.5)
project(intermediate_planner)

## Compile as C++14, supported in ROS Noetic and newer
set(CMAKE_CXX_FLAGS -std=c++14) # -Wextra -Werror

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages





add_executable(intermediate_planner src/intermediate_planner.cpp)
find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mapping REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_visualization REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(traj_planner REQUIRED)
find_package(path_search REQUIRED)
find_package(1.7 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_msgs REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${laser_geometry_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${mapping_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${plan_visualization_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${traj_planner_INCLUDE_DIRS} ${REQUIRED_INCLUDE_DIRS}
  ${path_search_INCLUDE_DIRS} ${1.7_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS} ${plan_msgs_INCLUDE_DIRS})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ./
)
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${laser_geometry_LIBRARIES}
  ${std_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${mapping_LIBRARIES}
  ${rclpy_LIBRARIES} ${geometry_msgs_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${plan_visualization_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES}
  ${traj_planner_LIBRARIES} ${REQUIRED_LIBRARIES} ${path_search_LIBRARIES}
  ${1.7_LIBRARIES} ${visualization_msgs_LIBRARIES} ${plan_msgs_LIBRARIES})
target_link_libraries(intermediate_planner
  ${cpp_typesupport_target}
  # ${SDL2_LIBRARIES}
  # ${OPENGL_LIBRARIES}
)
set(LIBS ${ament_cmake_LIBRARIES} ${laser_geometry_LIBRARIES}
  ${std_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${mapping_LIBRARIES}
  ${rclpy_LIBRARIES} ${geometry_msgs_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${plan_visualization_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES}
  ${traj_planner_LIBRARIES} ${REQUIRED_LIBRARIES} ${path_search_LIBRARIES}
  ${1.7_LIBRARIES} ${visualization_msgs_LIBRARIES} ${plan_msgs_LIBRARIES})


ament_export_dependencies(ament_cmake)
ament_export_dependencies(laser_geometry)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(mapping)
ament_export_dependencies(rclpy)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(plan_visualization)
ament_export_dependencies(Eigen3)
ament_export_dependencies(PCL)
ament_export_dependencies(traj_planner)
ament_export_dependencies(path_search)
ament_export_dependencies(1.7)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(plan_msgs)
ament_export_target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ./
)

ament_package()
