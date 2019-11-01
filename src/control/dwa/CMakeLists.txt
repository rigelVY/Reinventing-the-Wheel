cmake_minimum_required(VERSION 2.8.3)
project(dwa)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dwa_node src/dwa_node.cpp src/dwa.cpp)
target_link_libraries(dwa_node ${catkin_LIBRARIES})