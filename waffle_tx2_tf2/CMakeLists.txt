cmake_minimum_required(VERSION 3.8)
project(waffle_tx2_tf2)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS
  roscpp
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

add_executable(init_localization_tf2_node src/init_localization_tf2_node.cpp)
target_link_libraries(init_localization_tf2_node ${catkin_LIBRARIES})

add_executable(republish_imu_node src/republish_imu_node.cpp)
target_link_libraries(republish_imu_node ${catkin_LIBRARIES})

add_executable(tf2_odom_node src/tf2_odom_node.cpp)
target_link_libraries(tf2_odom_node ${catkin_LIBRARIES})

add_executable(tf2_static_node src/tf2_static_node.cpp)
target_link_libraries(tf2_static_node ${catkin_LIBRARIES})

add_executable(tf2_sync_odom_node src/tf2_sync_odom_node.cpp)
target_link_libraries(tf2_sync_odom_node ${catkin_LIBRARIES})
