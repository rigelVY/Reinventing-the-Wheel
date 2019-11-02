#include <waypoint_publisher/waypoint_publisher.h>

WaypointPublisher::WaypointPublisher(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("path_topic", path_topic_, "/waypoints/MissionA");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    wps_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 10);
    WaypointPublisher::WaypointPublisherArray_();
    boost::thread wp_publisher_thread(boost::bind(&WaypointPublisher::PublishWaypoints_, this));
}

WaypointPublisher::~WaypointPublisher()
{

}

void WaypointPublisher::WaypointPublisherArray_(void)
{
    current_time_ = ros::Time::now();
    wps_.header.stamp = current_time_;
    wps_.header.frame_id = map_frame_;
   
    for(int i=0; i<20; i++)
    {
        geometry_msgs::PoseStamped wp;
        CreatWaypoint_(i ,&wp);
        wps_.poses.push_back(wp);
    }
}       

void WaypointPublisher::CreatWaypoint_(int k, geometry_msgs::PoseStamped* wp)
{
    double x = k * 0.5;
    wp->pose.position.x = k;
    wp->pose.position.y = std::sin(x);
    wp->pose.position.z = 0;
    wp->header.stamp = current_time_;
    wp->header.frame_id = map_frame_;
   
} 
    
void WaypointPublisher::PublishWaypoints_(void)
{
    ros::Rate loop_late(10);
    while(ros::ok())
    {
        wps_pub_.publish(wps_);
        loop_late.sleep();
    }
    return;
}
