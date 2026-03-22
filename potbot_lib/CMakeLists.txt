cmake_minimum_required(VERSION 3.5)
project(potbot_lib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(potbot_lib_utility
  src/interpolate.cpp
  src/utility.cpp
  src/diff_drive_agent.cpp
  src/pid.cpp
  src/optimal_path_follower.cpp
  src/field.cpp
  src/artificial_potential_field.cpp
  src/apf_path_planner.cpp
  src/pure_pursuit.cpp
  src/time_state.cpp
)

ament_target_dependencies(potbot_lib_utility
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

target_include_directories(potbot_lib_utility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS potbot_lib_utility
  EXPORT export_potbot_lib_utility
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_potbot_lib_utility)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
ament_export_include_directories(include)

# ============================================================
# テスト
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_LIBS
    potbot_lib_utility
  )

  ament_add_gtest(test_utility test/test_utility.cpp)
  target_include_directories(test_utility PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_utility ${TEST_LIBS})

  ament_add_gtest(test_diff_drive_agent test/test_diff_drive_agent.cpp)
  target_include_directories(test_diff_drive_agent PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_diff_drive_agent ${TEST_LIBS})

  ament_add_gtest(test_pid test/test_pid.cpp)
  target_include_directories(test_pid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_pid ${TEST_LIBS})

  ament_add_gtest(test_field test/test_field.cpp)
  target_include_directories(test_field PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_field ${TEST_LIBS})

  ament_add_gtest(test_artificial_potential_field test/test_artificial_potential_field.cpp)
  target_include_directories(test_artificial_potential_field PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_artificial_potential_field ${TEST_LIBS})

  ament_add_gtest(test_apf_path_planner test/test_apf_path_planner.cpp)
  target_include_directories(test_apf_path_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_apf_path_planner ${TEST_LIBS})

  ament_add_gtest(test_optimal_path_follower test/test_optimal_path_follower.cpp)
  target_include_directories(test_optimal_path_follower PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_optimal_path_follower ${TEST_LIBS})

  ament_add_gtest(test_interpolate test/test_interpolate.cpp)
  target_include_directories(test_interpolate PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_interpolate ${TEST_LIBS})

  ament_add_gtest(test_diff_drive_agent_param test/test_diff_drive_agent_param.cpp)
  target_include_directories(test_diff_drive_agent_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_diff_drive_agent_param ${TEST_LIBS})

  ament_add_gtest(test_pid_param test/test_pid_param.cpp)
  target_include_directories(test_pid_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_pid_param ${TEST_LIBS})

  ament_add_gtest(test_apf_path_planner_param test/test_apf_path_planner_param.cpp)
  target_include_directories(test_apf_path_planner_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_apf_path_planner_param ${TEST_LIBS})

  ament_add_gtest(test_field_param test/test_field_param.cpp)
  target_include_directories(test_field_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_field_param ${TEST_LIBS})

  ament_add_gtest(test_apf_param test/test_apf_param.cpp)
  target_include_directories(test_apf_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_apf_param ${TEST_LIBS})

  add_subdirectory(test/integration)
  add_subdirectory(test/regression)
endif()

ament_package()
