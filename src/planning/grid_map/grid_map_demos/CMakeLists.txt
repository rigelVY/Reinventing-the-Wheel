cmake_minimum_required(VERSION 2.8.3)
project(grid_map_demos)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  grid_map_visualization
  geometry_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  simple_demo
  src/simple_demo_node.cpp
)

add_executable(
  image_to_gridmap_demo
  src/image_to_gridmap_demo_node.cpp
  src/ImageToGridmapDemo.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  simple_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  image_to_gridmap_demo
  ${catkin_LIBRARIES}
)

catkin_install_python(
  PROGRAMS scripts/image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(
  TARGETS simple_demo image_to_gridmap_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
