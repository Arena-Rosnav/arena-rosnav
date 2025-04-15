cmake_minimum_required(VERSION 3.10)
project(hunav_gz_plugin)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Include directories
include_directories(
  include
)

# Actor Plugin
add_library(HuNavActorPluginIGN SHARED src/HuNavActorPlugin.cpp)
target_include_directories(HuNavActorPluginIGN PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(HuNavActorPluginIGN 
  rclcpp
  tf2
  tf2_geometry_msgs
  hunav_msgs
  gz-plugin2
  gz-sim8
)
target_link_libraries(HuNavActorPluginIGN
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

# Install targets
install(TARGETS HuNavActorPluginIGN
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()