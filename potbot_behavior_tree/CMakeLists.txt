cmake_minimum_required(VERSION 3.5)
project(potbot_behavior_tree CXX)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(potbot_msgs REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(potbot_ros REQUIRED)

add_executable(rotate_to_goal_direction_server_node 
  src/rotate_to_goal_direction_server.cpp)
ament_target_dependencies(rotate_to_goal_direction_server_node
  rclcpp
  rclcpp_action
  potbot_msgs
)
install(TARGETS rotate_to_goal_direction_server_node
  DESTINATION lib/${PROJECT_NAME}
)

nav2_package()

include_directories(
  include
)

add_library(behavior_rotate_to_goal_direction SHARED
  plugins/rotate_to_goal_direction.cpp
)

ament_target_dependencies(behavior_rotate_to_goal_direction
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  nav2_util
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_core
  pluginlib
  potbot_msgs
  potbot_ros
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS behavior_rotate_to_goal_direction
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
  potbot_msgs
)

add_library(potbot_wait_action_bt_node SHARED plugins/action/wait_action.cpp)
list(APPEND plugin_libs potbot_wait_action_bt_node)

add_library(rotate_to_goal_direction_action_bt_node SHARED plugins/action/rotate_to_goal_direction_action.cpp)
list(APPEND plugin_libs rotate_to_goal_direction_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
  potbot_rotate_behavior
)

ament_export_dependencies(${dependencies})

ament_package()
