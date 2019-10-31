#ifndef WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED
#define WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <sstream>

class CheckpointSaver
{
public:
    CheckpointSaver(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~CheckpointSaver();
private:
    void InitializeCheckpointsFile_(void);
    void PublishPointsMarkerArray_(const geometry_msgs::PoseStamped pose);
    void CheckpointCallback_(const geometry_msgs::PoseStamped::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string cp_marker_topic_;
    std::string checkpoints_path_;
    std::string map_frame_;

    std::ofstream ofs_;
    
    ros::Subscriber checkpoint_sub_;
    ros::Publisher cp_marker_pub_;
};

#endif  //WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED

