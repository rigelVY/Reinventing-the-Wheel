#ifndef APPROXIMATE_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_INCLUDED
#define APPROXIMATE_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class ApproximateVoxelFilter
{
public:
    ApproximateVoxelFilter(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~ApproximateVoxelFilter();
private:
    void PointsCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void PublishFilteredPoints_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    double voxel_leaf_size_;
    std::string points_topic_;
    std::string filtered_points_topic_;

    ros::Publisher filtered_points_pub_;
    ros::Subscriber points_sub_;

    sensor_msgs::PointCloud2 points_raw_;
    pcl::PointCloud<pcl::PointXYZ> input_points_;
};

#endif  //APPROXIMATE_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_INCLUDED

