cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)

# Add Qt dependencies
set(QT_MAJOR_VERSION 5)
find_package(Qt${QT_MAJOR_VERSION} COMPONENTS Core Gui Widgets REQUIRED)

# Include Qt directories
include_directories(
  include
  ${Qt${QT_MAJOR_VERSION}Widgets_INCLUDE_DIRS}
  ${Qt${QT_MAJOR_VERSION}Gui_INCLUDE_DIRS}
  ${Qt${QT_MAJOR_VERSION}Core_INCLUDE_DIRS}
)

# Define QT_NO_KEYWORDS
add_definitions(-DQT_NO_KEYWORDS)

# Specify moc files
set(MOC_FILES
  src/goal_tool.h
  src/task_tool.h
)
qt5_wrap_cpp(MOC_FILES ${MOC_FILES})

# Specify source files
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/task_tool.cpp
  ${MOC_FILES}
)

# Create rviz_plugins library
add_library(${PROJECT_NAME} ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rviz_rendering
  rclcpp
  Qt${QT_MAJOR_VERSION}Core
  Qt${QT_MAJOR_VERSION}Gui
  Qt${QT_MAJOR_VERSION}Widgets
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

ament_package()
