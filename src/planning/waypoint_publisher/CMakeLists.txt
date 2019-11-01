cmake_minimum_required(VERSION 2.8.3)
project(waypoint_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)
 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_publisher
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_publisher_node src/waypoint_publisher_node.cpp src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher_node ${catkin_LIBRARIES})
