cmake_minimum_required(VERSION 2.8.3)
project(ndt_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.5 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_matching
  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ndt_matching_node src/ndt_matching_node.cpp src/ndt_matching.cpp)
target_link_libraries(ndt_matching_node ${catkin_LIBRARIES})

