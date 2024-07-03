cmake_minimum_required(VERSION 3.5)
project(gazebo_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt5Widgets REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(NOT Qt5Widgets_FOUND)
  message(FATAL_ERROR "Qt5Widgets not found")
endif()

# Specify additional locations of header files
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_default_plugins_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Define sources and headers
set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src/pose_tool.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src/goal_tool.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src/task_tool.cpp
)

# Wrap Qt meta-object compiler
qt5_wrap_cpp(MOC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src/goal_tool.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src/task_tool.h
)

# Create the library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${MOC_FILES})

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  Qt5Core
  Qt5Widgets
)

# Install targets and other files
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../rviz_plugins/src)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rviz_common rviz_default_plugins Qt5Widgets)

ament_package()

