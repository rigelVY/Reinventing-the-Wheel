cmake_minimum_required(VERSION 2.8.3)
project(localmap_2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(localmap_2d_node src/localmap_2d_node.cpp src/localmap_2d.cpp)
target_link_libraries(localmap_2d_node ${catkin_LIBRARIES})