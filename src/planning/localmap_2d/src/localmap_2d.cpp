#include <localmap_2d/localmap_2d.h>

LocalMap2D::LocalMap2D(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),map_(grid_map::GridMap({"occgrid_map", "cost_map"}))
{
    pnh_.param<std::string>("data_type", data_type_, "LaserScan");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("sensor_frame", sensor_frame_, "lidar_link");
    pnh_.param<std::string>("laser_topic", laser_topic_, "lidar_link/scan");
    pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "local_grid_map");
    pnh_.param<double>("map_width", map_width_, 1.2);
    pnh_.param<double>("map_height", map_height_, 2.0);
    pnh_.param<double>("resolution", resolution_, 0.05);
    pnh_.param<double>("obstacle_buffer", obstacle_buffer_, 1.0);
    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 1);
    laser_sub_ = nh_.subscribe(laser_topic_, 1, &LocalMap2D::LaserScanCallback_, this);

    LocalMap2D::GetTransformLidarToBase();

    map_.setFrameId(base_frame_);
    map_.setGeometry(grid_map::Length(map_width_, map_height_), resolution_);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map_.getLength().x(), map_.getLength().y(),
        map_.getSize()(0), map_.getSize()(1));
}

LocalMap2D::~LocalMap2D()
{

}

void LocalMap2D::GetTransformLidarToBase(void)
{
    tf::TransformListener listener;
    tf::StampedTransform tf_base_to_lidar;
    try
    {
        ros::Time now = ros::Time(0);
        listener.waitForTransform(base_frame_, sensor_frame_, now, ros::Duration(1.0));
        listener.lookupTransform(base_frame_, sensor_frame_, now, tf_base_to_lidar);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    lidar_offset_.x() = tf_base_to_lidar.getOrigin().x();
    lidar_offset_.y() = tf_base_to_lidar.getOrigin().y();
}

void LocalMap2D::LaserScanCallback_(const sensor_msgs::LaserScan::ConstPtr msg)
{
    map_.clearAll(); // initialize the map
    obstacles_pos_.clear();
    LocalMap2D::LaserScanToGridMap_(msg);
    LocalMap2D::GridMapToCostMap_();

    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    grid_map_pub_.publish(message);

    return;
}

void LocalMap2D::LaserScanToGridMap_(const sensor_msgs::LaserScan::ConstPtr laser_scan)
{
    for(int i=0; i<laser_scan->ranges.size(); i++)
    {
        double angle = laser_scan->angle_min + laser_scan->angle_increment * i;
        double range = laser_scan->ranges[i];
        if(range < laser_scan->range_min || range > laser_scan->range_max) continue;

        grid_map::Position position;
        position.x() = range * cos(angle) + lidar_offset_.x();
        position.y() = range * sin(angle) + lidar_offset_.y();
        if(map_width_/2.0 < abs(position.x()) || map_height_/2.0 < abs(position.y())) continue;
        map_.atPosition("occgrid_map", position) = 1.0;

        obstacles_pos_.emplace_back(position);
    }
    return;
}

void LocalMap2D::GridMapToCostMap_(void)
{
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) 
    {
        grid_map::Position position;
        map_.getPosition(*it, position);

        double obs_dist_square, nearest_obs_dist_square = 100.0;
        for(int i=0; i<obstacles_pos_.size(); i++)
        {
            obs_dist_square = pow((obstacles_pos_[i].x() - position.x()), 2) + pow((obstacles_pos_[i].y() - position.y()), 2);
            if(obs_dist_square < nearest_obs_dist_square) nearest_obs_dist_square = obs_dist_square;
        }
    
        map_.at("cost_map", *it) = obstacle_buffer_ / sqrt(nearest_obs_dist_square);
    }
    return;
}


/*void LocalMap2D::GridMapToRightMethodMap_(void)
{
     
}
*/
