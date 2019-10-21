#ifndef LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED
#define LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

class LocalMap2D
{
public:
    LocalMap2D(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~LocalMap2D();
private:
    void LaserScanCallback_(const sensor_msgs::LaserScan::ConstPtr msg);
    void LaserScanToGridMap_(const sensor_msgs::LaserScan::ConstPtr laser_scan);
    void GridMapToCostMap_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string data_type_;
    std::string sensor_frame_;
    std::string laser_topic_;
    std::string grid_map_topic_;
    double map_width_, map_height_;
    double resolution_;
    double costmap_grad_;

    grid_map::GridMap map_;
    
    ros::Publisher grid_map_pub_;
    ros::Subscriber laser_sub_;

    std::vector<grid_map::Index> obstacles_index_;
};

#endif  //LOCALMAP_2D_LOCALMAP_2D_H_INCLUDED

