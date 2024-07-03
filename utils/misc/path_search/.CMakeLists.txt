cmake_minimum_required(VERSION 3.5)
project(path_search)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS -std=c++14) # -Wextra -Werror



find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mapping REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${REQUIRED_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${mapping_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${nav_msgs_LIBRARIES} ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES} ${REQUIRED_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES} ${rclpy_LIBRARIES} ${std_msgs_LIBRARIES}
  ${mapping_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${nav_msgs_LIBRARIES} ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES} ${REQUIRED_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES} ${rclpy_LIBRARIES} ${std_msgs_LIBRARIES}
  ${mapping_LIBRARIES})

# this is important to find eigen path
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})




## Declare a C++ library
add_library(path_search src/astar.cpp src/kinodynamic_astar.cpp
  src/dyn_astar.cpp)






ament_export_dependencies(ament_cmake)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(PCL)
ament_export_dependencies(Eigen3)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(mapping)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(path_search ${LIBS})

ament_package()
