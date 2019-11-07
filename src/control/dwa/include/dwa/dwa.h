#ifndef DWA_DWA_H_INCLUDED
#define DWA_DWA_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <visualization_msgs/Marker.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class DWA
{
public:
    DWA(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~DWA();
private:
    struct path_point
    {
        double x;
        double y;
        double theta;
        double lin_v;
        double ang_v;
    };
    
    void WaypointsRawCallback_(const nav_msgs::Path::ConstPtr msg);
    void CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    void LocalGridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg);
    std::vector<DWA::path_point> CreatePath_(double linear_vel, double angular_vel);
    std::vector<std::vector<DWA::path_point>> CreateCandidatePaths_(void);
    double ObstacleCost_(std::vector<DWA::path_point> path);
    double LinearVelCost_(double linear_vel);
    double HeadingGoalCost_(DWA::path_point terminal_point);
    void PublishOptimizedPath_(std::vector<DWA::path_point> opt_path);
    double EvaluatePath_(void);
    bool ObstacleClearanceCheck_(geometry_msgs::Point target_position);
    void PublishTargetMarker_(geometry_msgs::PoseStamped target_pose);
    void PublishCmdVel_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string map_frame_;
    std::string twist_topic_;
    std::string path_topic_;
    std::string current_pose_topic_;
    std::string grid_map_topic_;
    std::string opt_path_topic_;
    std::string target_marker_topic_;
    int target_search_interval_;
    double linear_vel_max_;
    double linear_vel_min_;
    double linear_vel_resolution_;
    double angular_vel_max_;
    double angular_vel_min_;
    double angular_vel_resolution_;
    double path_time_length_;
    double path_time_resolution_;    
    double lookahead_dist_;
    double weight_obs_;
    double weight_vel_;
    double weight_angle_;
    double obs_avoid_threshold_;

    nav_msgs::Path wps_;
    geometry_msgs::PoseStamped current_pose_;
    grid_map::GridMap map_;
    
    ros::Publisher twist_pub_;
    ros::Publisher opt_path_pub_;
    ros::Publisher target_marker_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber local_gridmap_sub_;

    geometry_msgs::Point target_pos_;
    double optimal_linear_vel_, optimal_angular_vel_;

    bool current_pose_received_, waypoints_raw_received_, grid_map_received_;
};

#endif  //DWA_DWA_H_INCLUDED

