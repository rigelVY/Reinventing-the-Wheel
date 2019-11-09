cmake_minimum_required(VERSION 2.8.3)
project(fail_safe_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  grid_map_core
  grid_map_ros
  grid_map_msgs
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

add_executable(fail_safe_manager_node src/fail_safe_manager_node.cpp src/fail_safe_manager.cpp)
add_dependencies(fail_safe_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fail_safe_manager_node ${catkin_LIBRARIES})