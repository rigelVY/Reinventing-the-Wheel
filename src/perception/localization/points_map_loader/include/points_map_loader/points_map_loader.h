#ifndef POINTS_MAP_LOADER_POINTS_MAP_LOADER_H_INCLUDED
#define POINTS_MAP_LOADER_POINTS_MAP_LOADER_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class PointsMapLoader
{
public:
    PointsMapLoader(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PointsMapLoader();
private:
    void PublishPointsMap_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string map_path_;
    std::string map_frame_;
    std::string points_map_topic_;

    ros::Publisher points_map_pub_;

};

#endif  //POINTS_MAP_LOADER_POINTS_MAP_LOADER_H_INCLUDED

