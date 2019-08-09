cmake_minimum_required(VERSION 2.8.3)
project(dummy_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_localizer_node src/dummy_localizer_node.cpp src/dummy_localizer.cpp)
target_link_libraries(dummy_localizer_node ${catkin_LIBRARIES})