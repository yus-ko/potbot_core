cmake_minimum_required(VERSION 3.0.2)
project(potbot_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  navfn
  nav_msgs
  pluginlib
  roscpp
  tf2_ros
  sensor_msgs
  cv_bridge
  potbot_lib
  potbot_msgs
  potbot_base
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(OpenCV REQUIRED opencv)

generate_dynamic_reconfigure_options(
  cfg/StatePlugin.cfg
  cfg/PID.cfg
  cfg/PurePursuit.cfg
  cfg/TimeState.cfg
  cfg/OptimalPathFollower.cfg
  cfg/TraditionalAPF.cfg
  cfg/ParticleSwarmOptimization.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} potbot_lib test_pkg
  # CATKIN_DEPENDS
  #   costmap_2d
  #   dynamic_reconfigure
  #   geometry_msgs
  #   nav_core
  #   navfn
  #   nav_msgs
  #   pluginlib
  #   roscpp
  #   tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/potbot_plan.cpp
  src/local_planner.cpp
  src/state_layer.cpp
  src/pid.cpp
  src/pure_pursuit.cpp
  src/time_state.cpp
  src/optimal_path_follower.cpp
  src/traditional_apf.cpp
  src/apfpp.cpp
  src/particle_swarm_optimization.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_definitions(-std=c++14)
