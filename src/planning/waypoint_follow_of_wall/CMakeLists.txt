cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follow_of_wall)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  grid_map_ros
  grid_map_core
  grid_map_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_follow_of_wall
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf grid_map_ros grid_map_core grid_map_msgs 
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_follow_of_wall_node src/waypoint_follow_of_wall.cpp src/waypoint_follow_of_wall_node.cpp)
target_link_libraries(waypoint_follow_of_wall_node ${catkin_LIBRARIES})
