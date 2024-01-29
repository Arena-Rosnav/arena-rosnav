cmake_minimum_required(VERSION 3.0.2)
project(intermediate_planner)

## Compile as C++14, supported in ROS Noetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  
  visualization_msgs
  plan_visualization

  mapping
  path_search
  traj_planner
  plan_msgs
  laser_geometry
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES intermediate_planner
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    
    visualization_msgs
    plan_visualization

    mapping
    path_search
    traj_planner
    plan_msgs
    laser_geometry
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(intermediate_planner 
              src/intermediate_planner.cpp
)

target_link_libraries(intermediate_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
