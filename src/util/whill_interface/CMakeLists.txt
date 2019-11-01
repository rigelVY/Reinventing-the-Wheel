cmake_minimum_required(VERSION 2.8.3)
project(whill_interface)

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

add_executable(whill_interface_node src/whill_interface_node.cpp src/whill_interface.cpp)
add_dependencies(whill_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(whill_interface_node ${catkin_LIBRARIES})