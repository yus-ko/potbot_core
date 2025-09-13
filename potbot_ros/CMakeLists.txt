cmake_minimum_required(VERSION 3.5)
project(potbot_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(potbot_lib REQUIRED)

include_directories(include)

add_library(potbot_ros_utility
  src/utility.cpp
  src/interactive_marker_manager.cpp
)

ament_target_dependencies(potbot_ros_utility
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  interactive_markers
  potbot_lib
)

target_include_directories(potbot_ros_utility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS potbot_ros_utility
  EXPORT export_potbot_ros_utility
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_potbot_ros_utility)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  interactive_markers
  potbot_lib
)
ament_export_include_directories(include)

ament_package()
