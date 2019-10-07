#include <localmap_2d/localmap_2d.h>

LocalMap2D::LocalMap2D(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),map_(grid_map::GridMap({"occgrid_map", "cost_map"}))
{
    pnh_.param<std::string>("data_type", data_type_, "LaserScan");
    pnh_.param<std::string>("sensor_frame", sensor_frame_, "lidar_link");
    pnh_.param<std::string>("laser_topic", laser_topic_, "lidar_link/scan");
    pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "waypoints_raw");
    pnh_.param<double>("map_width", map_width_, 1.2);
    pnh_.param<double>("map_height", map_height_, 2.0);
    pnh_.param<double>("resolution", resolution_, 0.05);
    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 1);
    laser_sub_ = nh_.subscribe(laser_topic_, 1, &LocalMap2D::LaserScanCallback_, this);
    boost::thread publish_thread(boost::bind(&LocalMap2D::PublishCmdVel_, this));

    map_.setFrameId(sensor_frame_);
    map_.setGeometry(grid_map::Length(map_width_, map_height_), resolution_);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map_.getLength().x(), map_.getLength().y(),
        map_.getSize()(0), map_.getSize()(1));
}

LocalMap2D::~LocalMap2D()
{

}

void LocalMap2D::LaserScanCallback_(const sensor_msgs::LaserScan::ConstPtr msg)
{
    LocalMap2D::LaserScanToGridMap_(msg);
    LocalMap2D::GridMapToCostMap_();
    return;
}

void LocalMap2D::LaserScanToGridMap_(const sensor_msgs::LaserScan::ConstPtr laser_scan)
{
    map_.clearAll(); // initialize the map
    for(int i=0; i<laser_scan->ranges.size(); i++)
    {
        double angle = laser_scan->angle_min + laser_scan->angle_increment * i;
        double range = laser_scan->ranges[i];
        if(range < laser_scan->range_min || range > laser_scan->range_max) continue;

        grid_map::Position position;
        position.x() = range * cos(angle);
        position.y() = range * sin(angle);
        if(map_width_/2 < abs(position.x()) || map_height_/2 < abs(position.y())) continue;
        grid_map::Index index;
        map_.getIndex(position, index);
        map_.at("occgrid_map", index) = 0.5;
    }
    return;
}

void LocalMap2D::GridMapToCostMap_(void)
{
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) 
    {
        grid_map::Position position;
        map_.getPosition(*it, position);
        map_.at("cost_map", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
    }
}

void LocalMap2D::PublishCmdVel_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        map_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map_, message);
        grid_map_pub_.publish(message);

        loop_rate.sleep();
    }
    return;
}
