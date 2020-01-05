cmake_minimum_required(VERSION 3.8)
project(waffle_tx2_navigation)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)
add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES})

