cmake_minimum_required(VERSION 3.5)
project(potbot_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
#   navfn
#   sensor_msgs
#   cv_bridge
find_package(potbot_lib REQUIRED)
find_package(potbot_ros REQUIRED)
#   potbot_msgs

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME} potbot_lib test_pkg
#   # CATKIN_DEPENDS
#   #   costmap_2d
#   #   dynamic_reconfigure
#   #   geometry_msgs
#   #   nav_core
#   #   navfn
#   #   nav_msgs
#   #   pluginlib
#   #   roscpp
#   #   tf2_ros
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

# add_library(${PROJECT_NAME}
#   src/potbot_plan.cpp
#   src/local_planner.cpp
#   src/state_layer.cpp
#   src/pid.cpp
#   src/pure_pursuit.cpp
#   src/time_state.cpp
#   src/optimal_path_follower.cpp
#   src/traditional_apf.cpp
#   src/apfpp.cpp
#   src/particle_swarm_optimization.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  potbot_lib
  potbot_ros
)

add_library(potbot_plugin SHARED
  src/optimal_path_follower.cpp)

# prevent pluginlib from using boost
target_compile_definitions(potbot_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(potbot_plugin
  ${dependencies}
)

target_include_directories(potbot_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS potbot_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(potbot_plugin)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin_nav2_core.xml)

ament_package()
