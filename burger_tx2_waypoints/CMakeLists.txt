cmake_minimum_required(VERSION 3.8)
project(burger_tx2_waypoints)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
