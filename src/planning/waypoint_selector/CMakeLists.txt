cmake_minimum_required(VERSION 2.8.3)
project(waypoint_selector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostate_machine
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_selector_node src/waypoint_selector_node.cpp src/waypoint_selector.cpp)
add_dependencies(waypoint_selector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_selector_node ${catkin_LIBRARIES})