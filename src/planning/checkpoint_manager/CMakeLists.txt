cmake_minimum_required(VERSION 2.8.3)
project(checkpoint_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  checkpoint_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(checkpoint_manager_node src/checkpoint_manager_node.cpp src/checkpoint_manager.cpp)
target_link_libraries(checkpoint_manager_node ${catkin_LIBRARIES})