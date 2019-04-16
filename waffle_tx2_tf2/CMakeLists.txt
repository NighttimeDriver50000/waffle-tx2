cmake_minimum_required(VERSION 3.8)
project(waffle_tx2_tf2)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_odom_node src/tf2_odom_node.cpp)
target_link_libraries(tf2_odom_node ${catkin_LIBRARIES})

add_executable(tf2_imu_node src/tf2_imu_node.cpp)
target_link_libraries(tf2_imu_node ${catkin_LIBRARIES})

add_executable(tf2_scan_node src/tf2_scan_node.cpp)
target_link_libraries(tf2_scan_node ${catkin_LIBRARIES})

add_executable(tf2_zed_node src/tf2_zed_node.cpp)
target_link_libraries(tf2_zed_node ${catkin_LIBRARIES})

add_executable(tf2_zed_optical_node src/tf2_zed_optical_node.cpp)
target_link_libraries(tf2_zed_optical_node ${catkin_LIBRARIES})
