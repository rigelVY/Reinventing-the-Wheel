#ifndef LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED
#define LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class LocalMap2D
{
public:
    LocalMap2D(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~LocalMap2D();
private:
    void PublishCmdVel_(void);
    void LaserScanCallback_(const sensor_msgs::LaserScan::ConstPtr msg);
    void LaserScanToGridMap_(const sensor_msgs::LaserScan::ConstPtr laser_scan);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string data_type_;
    std::string sensor_frame_;
    std::string laser_topic_;
    std::string grid_map_topic_;
    double map_width_, map_height_;
    double resolution_;

    grid_map::GridMap map_;
    
    ros::Publisher grid_map_pub_;
    ros::Subscriber laser_sub_;
};

#endif  //LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED

