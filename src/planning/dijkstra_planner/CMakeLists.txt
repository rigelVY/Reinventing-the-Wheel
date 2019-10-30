cmake_minimum_required(VERSION 2.8.3)
project(dijkstra_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dijkstra_planner
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dijkstra_planner_node src/dijkstra_planner_node.cpp src/dijkstra_planner.cpp)
target_link_libraries(dijkstra_planner_node ${catkin_LIBRARIES})

