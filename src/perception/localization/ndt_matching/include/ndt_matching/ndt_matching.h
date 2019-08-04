#ifndef NDT_MATCHING_NDT_MATCHING_H_INCLUDED
#define NDT_MATCHING_NDT_MATCHING_H_INCLUDED

#include <ros/ros.h>
//#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/static_transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NdtMatching
{
public:
    NdtMatching(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~NdtMatching();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};

#endif  //NDT_MATCHING_NDT_MATCHING_H_INCLUDED
