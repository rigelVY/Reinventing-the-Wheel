#ifndef WAYPOINT_FOLLOW_OF_WALL_H_WAYPOINT_FOLLOW_OF_WALL_H_INCLUDED
#define WAYPOINT_FOLLOW_OF_WALL_H_WAYPOINT_FOLLOW_OF_WALL_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

//header in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

class WaypointFollowOfWall
{
public:
    WaypointFollowOfWall(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~WaypointFollowOfWall();
private:
    void GridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg);
    void WaypointFollowOfWallAllay_();
    void WaypointFollowOfWall_(grid_map::GridMap map_, geometry_msgs::PoseStamped* wp);
    void WaypointPublisher_(void);
   

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Time current_time_;

    std::string path_topic_;
    std::string map_frame_;
    std::string grid_map_topic_;
    nav_msgs::Path wps_;
    grid_map::GridMap map_;
 
    ros::Publisher wps_pub_;    
    ros::Subscriber grid_map_sub_;

};

#endif //WAYPOINT_FOLLOW_OF_WALL_H_INCLUDED  
