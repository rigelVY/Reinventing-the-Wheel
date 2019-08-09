#ifndef DUMMY_LOCALIZER_DUMMY_LOCALIZER_H_INCLUDED
#define DUMMY_LOCALIZER_DUMMY_LOCALIZER_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
//#include <visualization_msgs/MarkerArray.h>
// #include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class DummyLocalizer
{
public:
    DummyLocalizer(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~DummyLocalizer();
private:
    void PublishCurrentPoseStamped_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string base_frame_;
    std::string map_frame_;
    std::string pose_topic_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher current_pose_pub_;
};

#endif  //DUMMY_LOCALIZER_DUMMY_LOCALIZER_H_INCLUDED
