cmake_minimum_required(VERSION 2.8.3)
project(roomba_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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

add_executable(roomba_interface_node src/roomba_interface_node.cpp src/roomba_interface.cpp)
add_dependencies(roomba_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roomba_interface_node ${catkin_LIBRARIES})