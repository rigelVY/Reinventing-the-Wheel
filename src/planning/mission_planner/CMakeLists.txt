cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
  rostate_machine
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_planner_node src/mission_planner_node.cpp src/mission_planner.cpp)
add_dependencies(mission_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_planner_node ${catkin_LIBRARIES})