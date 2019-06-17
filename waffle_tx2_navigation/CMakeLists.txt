cmake_minimum_required(VERSION 3.8)
project(waffle_tx2_navigation)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES})

