#include <dwa/dwa.h>

DWA::DWA(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");
    pnh_.param<std::string>("path_topic", path_topic_, "waypoints_raw");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<double>("linear_velocity", linear_velocity_, 0.1);
    pnh_.param<double>("lookahead_distance", lookahead_dist_, 0.5);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 1);
    path_sub_ = nh_.subscribe(path_topic_, 1, &DWA::WaypointsRawCallback_, this);
    current_pose_sub_ = nh_.subscribe(current_pose_topic_, 1, &DWA::CurrentPoseCallback_, this);
    boost::thread publish_thread(boost::bind(&DWA::PublishCmdVel_, this));
}

DWA::~DWA()
{

}

void DWA::WaypointsRawCallback_(const nav_msgs::Path::ConstPtr msg)
{
    wps_ = *msg;
    return;
}

void DWA::CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose_ = *msg;
    return;
}

void DWA::PublishCmdVel_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        int target_index;
        geometry_msgs::Point target_relative_pos;
        double target_relative_dist, target_relative_angle;

        for(target_index=0; target_index<wps_.poses.size(); target_index++)
        {
            
            target_relative_pos.x = wps_.poses[target_index].pose.position.x - current_pose_.pose.position.x;
            target_relative_pos.y = wps_.poses[target_index].pose.position.y - current_pose_.pose.position.y;
            target_relative_pos.z = wps_.poses[target_index].pose.position.z - current_pose_.pose.position.z;
            target_relative_dist = std::sqrt(std::pow(target_relative_pos.x, 2) + std::pow(target_relative_pos.y, 2) + std::pow(target_relative_pos.z, 2));
            target_relative_angle = std::atan2(target_relative_pos.y, target_relative_pos.x) - tf::getYaw(current_pose_.pose.orientation);

            if(std::abs(target_relative_angle) < M_PI/2)
            {
                if(target_relative_dist > lookahead_dist_) break;
            }
        }

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity_;
        cmd_vel.angular.z = 2.0 * linear_velocity_ * sin(target_relative_angle) / target_relative_dist;
        twist_pub_.publish(cmd_vel);

        loop_rate.sleep();
    }
    return;
}
