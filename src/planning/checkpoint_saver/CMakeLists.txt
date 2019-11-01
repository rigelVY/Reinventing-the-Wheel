cmake_minimum_required(VERSION 2.8.3)
project(checkpoint_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES checkpoint_saver
  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(checkpoint_saver_node src/checkpoint_saver_node.cpp src/checkpoint_saver.cpp)
target_link_libraries(checkpoint_saver_node ${catkin_LIBRARIES})

