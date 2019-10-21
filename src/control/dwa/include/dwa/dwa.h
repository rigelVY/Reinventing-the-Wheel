#ifndef DWA_DWA_H_INCLUDED
#define DWA_DWA_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

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
    std::vector<DWA::path_point> CreatePath_(double linear_vel, double angular_vel, double dt, double move_time);
    std::vector<std::vector<DWA::path_point>> CreateCandidatePaths_(double dt, double move_time, double resolution);
    double ObstacleCost_(std::vector<DWA::path_point> path);
    double LinearVelCost_(double linear_vel);
    double HeadingGoalCost_(double terminal_angle);
    void PublishOptimizedPath_(std::vector<DWA::path_point> opt_path);
    double EvaluatePath_(double weight_obs, double weight_vel, double weight_angle, double dt, double move_time);
    void PublishCmdVel_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string twist_topic_;
    std::string path_topic_;
    std::string current_pose_topic_;
    std::string grid_map_topic_;
    std::string opt_path_topic_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double min_angular_vel_;
    double lookahead_dist_;

    nav_msgs::Path wps_;
    geometry_msgs::PoseStamped current_pose_;
    grid_map::GridMap map_;
    
    ros::Publisher twist_pub_;
    ros::Publisher opt_path_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber local_gridmap_sub_;

    double target_relative_dist_, target_relative_angle_;
    double optimal_linear_vel_, optimal_angular_vel_;

    int current_pose_received_;
};

#endif  //DWA_DWA_H_INCLUDED

