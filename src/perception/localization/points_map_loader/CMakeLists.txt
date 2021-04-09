cmake_minimum_required(VERSION 2.8.3)
project(points_map_loader)

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
  LIBRARIES points_map_loader
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(points_map_loader_node src/points_map_loader_node.cpp src/points_map_loader.cpp)
target_link_libraries(points_map_loader_node ${catkin_LIBRARIES})
