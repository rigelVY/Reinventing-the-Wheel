#include <ros/ros.h>
#include "grid_map_demos/ImageToGridmapDemo.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap_demo");
  ros::NodeHandle nh("~");
  ImageToGridmapDemo imageToGridmapDemo(nh);

  ros::spin();
  return 0;
}
