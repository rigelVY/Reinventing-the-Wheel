#ifndef CHECKPOINT_MANAGER_CHECKPOINT_MANAGER_H_INCLUDED
#define CHECKPOINT_MANAGER_CHECKPOINT_MANAGER_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <checkpoint_msgs/StateArray.h>
#include <checkpoint_msgs/ResetMsg.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <fstream>
#include <sstream>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class CheckpointManager
{
public:
    CheckpointManager(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~CheckpointManager();
private:
    void InitializeCheckpointArray_(void);
    void LoadCheckpoint_(const std::string& line, checkpoint_msgs::State* state);
    void CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    void ResetMsgCallback_(const checkpoint_msgs::ResetMsg::ConstPtr msg);
    void PublishCheckpointMarker_(void);
    void PublishCheckpointArray_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Time current_time_;
    
    std::string checkpoints_path_;
    std::string state_array_topic_;
    std::string marker_topic_;
    std::string current_pose_topic_;
    std::string reset_topic_;
    std::string map_frame_;
    double boundary_distance_;

    geometry_msgs::PoseStamped current_pose_;
    checkpoint_msgs::StateArray cp_states_;
    
    ros::Subscriber current_pose_sub_;
    ros::Subscriber reset_sub_;
    ros::Publisher cp_states_pub_;
    ros::Publisher markers_pub_;
};

#endif  //CHECKPOINT_MANAGER_CHECKPOINT_MANAGER_H_INCLUDED

