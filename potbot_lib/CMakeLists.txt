cmake_minimum_required(VERSION 3.0.2)
project(potbot_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  potbot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS
  DEPENDS eigen #opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Utility.cpp
  src/PotentialField.cpp
  src/PathPlanner.cpp
  src/UnscentedKalmanFilter.cpp
  src/PCLClustering.cpp
  src/ScanClustering.cpp
  src/DiffDriveController.cpp
  src/DWAController.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++14)