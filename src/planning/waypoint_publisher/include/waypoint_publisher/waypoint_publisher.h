#ifndef WAYPOINT_PUBLISHER_WAYPOINT_PUBLISHER_H_INCLUDED
#define WAYPOINT_PUBLISHER_WAYPOINT_PUBLISHER_H_INCLUDED 


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

//header in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


class WaypointPublisher
{
public:
    WaypointPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~WaypointPublisher();
private:
    void PublishWaypoints_(void);
    void CreatWaypoint_(int k, geometry_msgs::PoseStamped* wp);
    void WaypointPublisherArray_(void);    
   
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Time current_time_;

    std::string path_topic_;
    std::string map_frame_;
    nav_msgs::Path wps_;
    
    ros::Publisher wps_pub_;
};


#endif //WAYPOINT_PUBLISHER_WAYPOINT_PUBLISHER_H_INCLUDED

