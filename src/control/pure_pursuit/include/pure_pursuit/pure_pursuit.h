#ifndef PURE_PURSUIT_PURE_PURSUIT_H_INCLUDED
#define PURE_PURSUIT_PURE_PURSUIT_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class PurePursuit
{
public:
    PurePursuit(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PurePursuit();
private:
    void PublishCmdVel_(void);
    void WaypointsRawCallback_(const nav_msgs::Path::ConstPtr msg);
    void CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string twist_topic_;
    std::string path_topic_;
    std::string current_pose_topic_;
    double linear_velocity_;
    double lookahead_dist_;

    nav_msgs::Path wps_;
    geometry_msgs::PoseStamped current_pose_;
    
    ros::Publisher twist_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber current_pose_sub_;
};

#endif  //PURE_PURSUIT_PURE_PURSUIT_H_INCLUDED

