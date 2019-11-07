#include <waypoint_follow_of_wall/waypoint_follow_of_wall.h>

WaypointFollowOfWall::WaypointFollowOfWall (ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("path_topic", path_topic_, "/waypoints/MissionA");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("gridmap_topic", grid_map_topic_, "/gridmap");
    wps_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 10);
    grid_map_sub_ = nh.subscribe(grid_map_topic_, 1, &WaypointFollowOfWall::GridMapCallback_, this);
    WaypointFollowOfWall::WaypointFollowOfWallAllay_();
    boost::thread publish_thread(boost::bind(&WaypointFollowOfWall::WaypointPublisher_, this));
}

WaypointFollowOfWall::~WaypointFollowOfWall()
{

}

void WaypointFollowOfWall::GridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    return;
}

void WaypointFollowOfWall::WaypointFollowOfWallAllay_(void)
{
    current_time_ = ros::Time::now();
    wps_.header.stamp = current_time_;
    wps_.header.frame_id = map_frame_;
    
    while(!map_.getSize().x())
    {
        geometry_msgs::PoseStamped wp;
        WaypointFollowOfWall_(map_, &wp);
        wps_.poses.push_back(wp);
    }
}

void WaypointFollowOfWall::WaypointFollowOfWall_(grid_map::GridMap map_, geometry_msgs::PoseStamped* wp)
{
    grid_map::Index index;
    wp->pose.position.x = map_.getSize().y();
    wp->pose.position.y = 10 + map_.at("occgrid_map", index);
    wp->pose.position.z = 0;
    wp->header.stamp = current_time_;
    wp->header.frame_id = map_frame_;
}

void WaypointFollowOfWall::WaypointPublisher_(void)
{
    ros::Rate loop_late(10);
    while(ros::ok())
    {
        wps_pub_.publish(wps_);
        loop_late.sleep();
    }
    return;
}    
