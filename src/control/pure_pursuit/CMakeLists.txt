cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit_node ${catkin_LIBRARIES})