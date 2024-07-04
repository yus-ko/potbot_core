cmake_minimum_required(VERSION 3.0.2)
project(potbot_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  pluginlib
  pcl_ros
  potbot_msgs
  interactive_markers
  message_generation
)

add_service_files(
  FILES
  Save.srv
)

generate_dynamic_reconfigure_options(
  cfg/APFPathPlanner.cfg
  cfg/PotentialField.cfg
  cfg/Controller.cfg
  cfg/DWA.cfg
  cfg/MarkerManager.cfg
)

generate_messages(
  # DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS
  DEPENDS eigen #opencv
  CATKIN_DEPENDS
    costmap_2d
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/utility.cpp
  src/utility_ros.cpp
  src/interpolate.cpp
  src/field.cpp
  src/artificial_potential_field.cpp
  src/artificial_potential_field_ros.cpp
  src/apf_path_planner.cpp
  src/apf_path_planner_ros.cpp
  src/unscented_kalman_filter.cpp
  src/pcl_clustering.cpp
  src/scan_clustering.cpp
  src/diff_drive_agent.cpp
  src/diff_drive_controller.cpp
  src/diff_drive_controller_ros.cpp
  src/dwa_controller_ros.cpp
  src/interactive_marker_manager.cpp
  src/base_controller.cpp
  src/pure_pursuit.cpp
  src/dynamic_window_approach.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++14)