cmake_minimum_required(VERSION 3.0.2)
project(potbot_base)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
)

catkin_package(INCLUDE_DIRS include)
