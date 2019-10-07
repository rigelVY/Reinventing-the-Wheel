#include <ros/ros.h>
// #include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

class ImageToGridmapDemo
{
public:
  ImageToGridmapDemo(ros::NodeHandle& nodeHandle);

  virtual ~ImageToGridmapDemo();

  bool readParameters();
  void imageCallback(const sensor_msgs::Image& msg);

 private:
  ros::NodeHandle& nodeHandle_;
  ros::Publisher gridMapPublisher_;

  grid_map::GridMap map_;


  ros::Subscriber imageSubscriber_;

  std::string imageTopic_;


  double mapLengthX_;
  double resolution_;
  double minHeight_;
  double maxHeight_;

  std::string mapFrameId_;

  bool mapInitialized_;
};
