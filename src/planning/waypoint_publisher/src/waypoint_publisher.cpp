#include <waypoint_publisher/waypoint_publisher.h>

WaypointPublisher::WaypointPublisher(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("path_topic", path_topic_, "/waypoints/MissionA");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    wps_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 10);
    WaypointPublisher::CreatWaypoint_();
    boost::thread wp_publisher_thread(boost::bind(&WaypointPublisher::PublishWaypoints_, this));
}

WaypointPublisher::~WaypointPublisher()
{

}

void WaypointPublisher::CreatWaypoint_(void)
{
    for(int i=0; i<20; i++)
    {
        double x = i * 0.5;
        current_time_ = ros::Time::now();
        wps_.header.stamp = current_time_;
        wps_.header.frame_id = map_frame_;
        geometry_msgs::PoseStamped wp;

        wp.pose.position.x = i;
        wp.pose.position.y = std::sin(x);
        wp.pose.position.z = 0;
        wp.header.stamp = current_time_;
        wp.header.frame_id = map_frame_;
        wps_.poses.push_back(wp);
    }
    return;
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
