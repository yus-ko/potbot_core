cmake_minimum_required(VERSION 3.5)
project(potbot_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(yolo_msgs REQUIRED)

find_package(potbot_lib REQUIRED)
# find_package(potbot_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  # src/potbot_plan.cpp
  # src/local_planner.cpp
  # src/state_layer.cpp
  # src/pid.cpp
  # src/pure_pursuit.cpp
  # src/time_state.cpp
  # src/optimal_path_follower.cpp
  # src/traditional_apf.cpp
  # src/apfpp.cpp
  # src/particle_swarm_optimization.cpp
  src/human_pose_layer.cpp
)

include_directories(include)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_costmap_2d.xml)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  yolo_msgs
)

ament_package()
