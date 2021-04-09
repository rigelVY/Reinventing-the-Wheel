cmake_minimum_required(VERSION 2.8.3)
project(approximate_voxel_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.5 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES approximate_voxel_filter
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(approximate_voxel_filter_node src/approximate_voxel_filter_node.cpp src/approximate_voxel_filter.cpp)
target_link_libraries(approximate_voxel_filter_node ${catkin_LIBRARIES})
