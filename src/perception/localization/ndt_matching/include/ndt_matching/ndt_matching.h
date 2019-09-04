#ifndef NDT_MATCHING_NDT_MATCHING_H_INCLUDED
#define NDT_MATCHING_NDT_MATCHING_H_INCLUDED

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class NdtMatching
{
public:
    NdtMatching(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~NdtMatching();
private:
    void MapCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void InitialposeCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void PointsCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void PublishNDTPose_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    double trans_eps_;
    double step_size_;
    double ndt_res_;
    double max_iter_;
    std::string lidar_frame_;
    std::string map_frame_;
    std::string points_topic_;
    std::string map_topic_;
    std::string ndt_pose_topic_;

    ros::Publisher ndt_pose_pub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Subscriber filtered_points_sub_;

    pcl::PointCloud<pcl::PointXYZ> map_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    sensor_msgs::PointCloud2 filtered_points_;

    int map_loaded_;
    int init_pos_set_;
    unsigned int points_map_num_;
    
    Eigen::Matrix4f init_guess_;
    geometry_msgs::Pose initial_pose_;
};

#endif  //NDT_MATCHING_NDT_MATCHING_H_INCLUDED
