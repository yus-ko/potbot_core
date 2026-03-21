cmake_minimum_required(VERSION 3.5)
project(potbot_ros)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(potbot_lib REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(potbot_ros_utility
  src/utility.cpp
  src/interactive_marker_manager.cpp
  src/artificial_potential_field.cpp
  src/apf_path_planner.cpp
)

ament_target_dependencies(potbot_ros_utility
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  interactive_markers
  pcl_conversions
  nav2_costmap_2d
  potbot_lib
)
target_link_libraries(potbot_ros_utility yaml-cpp)

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
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  interactive_markers
  pcl_conversions
  nav2_costmap_2d
  potbot_lib
  yaml-cpp
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ros_utility test/test_ros_utility.cpp)
  target_include_directories(test_ros_utility PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_ros_utility potbot_ros_utility)
  ament_target_dependencies(test_ros_utility
    rclcpp geometry_msgs nav_msgs sensor_msgs)
endif()

ament_package()
