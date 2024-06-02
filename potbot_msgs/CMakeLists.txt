cmake_minimum_required(VERSION 3.0.2)
project(potbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  pcl_ros
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
  cfg/Clustering.cfg
  cfg/Localization.cfg
  cfg/Localmap.cfg
  cfg/PathPlanning.cfg
  cfg/ClusteringParam.cfg
)

add_action_files(
  FILES
  Navigation.action
)

add_message_files(
  FILES
  State.msg
  StateArray.msg
  ObstacleArray.msg
  Obstacle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
add_definitions(-std=c++14)