#include <dwa/dwa.h>

DWA::DWA(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),current_pose_received_(false),waypoints_raw_received_(false),grid_map_received_(false)
{
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");
    pnh_.param<std::string>("path_topic", path_topic_, "waypoints_raw");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "local_grid_map");
    pnh_.param<std::string>("opt_path_topic", opt_path_topic_, "dwa/opt_path_points");
    pnh_.param<std::string>("target_marker_topic", target_marker_topic_, "dwa/target_point");
    pnh_.param<int>("target_search_interval", target_search_interval_, 10);
    pnh_.param<double>("linear_vel_max", linear_vel_max_, 1.0);
    pnh_.param<double>("linear_vel_min", linear_vel_min_, 0.05);
    pnh_.param<double>("linear_vel_resolution", linear_vel_resolution_, 0.1);
    pnh_.param<double>("angular_vel_max", angular_vel_max_, 0.5);
    pnh_.param<double>("angular_vel_min", angular_vel_min_, -0.5);
    pnh_.param<double>("angular_vel_resolution", angular_vel_resolution_, 0.1);
    pnh_.param<double>("path_time_length", path_time_length_, 0.4);
    pnh_.param<double>("path_time_resolution", path_time_resolution_, 0.04);
    pnh_.param<double>("lookahead_distance", lookahead_dist_, 0.5);
    pnh_.param<double>("weight_obs", weight_obs_, 1.0);
    pnh_.param<double>("weight_vel", weight_vel_, 1.0);
    pnh_.param<double>("weight_angle", weight_angle_, 1.0);
    pnh_.param<double>("obs_avoid_threshold", obs_avoid_threshold_, 0.6);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 1);
    opt_path_pub_ = nh_.advertise<nav_msgs::Path>(opt_path_topic_, 1);
    target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(target_marker_topic_, 1);
    path_sub_ = nh_.subscribe(path_topic_, 1, &DWA::WaypointsRawCallback_, this);
    current_pose_sub_ = nh_.subscribe(current_pose_topic_, 1, &DWA::CurrentPoseCallback_, this);
    local_gridmap_sub_= nh_.subscribe(grid_map_topic_, 1, &DWA::LocalGridMapCallback_, this);
    boost::thread publish_thread(boost::bind(&DWA::PublishCmdVel_, this));
}

DWA::~DWA()
{

}

void DWA::WaypointsRawCallback_(const nav_msgs::Path::ConstPtr msg)
{
    wps_ = *msg;
    waypoints_raw_received_ = true;
    return;
}

void DWA::CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose_ = *msg;
    current_pose_received_ = true;
    return;
}

void DWA::LocalGridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    grid_map_received_ = true;
    return;
}

std::vector<DWA::path_point> DWA::CreatePath_(double linear_vel, double angular_vel)
{
    std::vector<DWA::path_point> path;
    path.reserve(std::ceil(path_time_length_/path_time_resolution_));
    DWA::path_point point;
    point.x = current_pose_.pose.position.x;
    point.y = current_pose_.pose.position.y;
    point.theta = tf::getYaw(current_pose_.pose.orientation);
    point.lin_v = linear_vel;
    point.ang_v = angular_vel;

    for(double t=0.0; t<path_time_length_; t+=path_time_resolution_)
    {
        point.x += linear_vel * cos(point.theta) * t;
        point.y += linear_vel * sin(point.theta) * t;
        point.theta += angular_vel * t;
        path.emplace_back(point);
    }
    return path;
}

std::vector<std::vector<DWA::path_point>> DWA::CreateCandidatePaths_(void)
{
    std::vector<std::vector<DWA::path_point>> paths;
    for(double lin_v=linear_vel_min_; lin_v<linear_vel_max_; lin_v+=linear_vel_resolution_)
    {
        for(double ang_v=angular_vel_min_; ang_v<angular_vel_max_; ang_v+=angular_vel_resolution_)
        {
            paths.emplace_back(CreatePath_(lin_v, ang_v));
        }
    }
    return paths;
}

double DWA::ObstacleCost_(std::vector<DWA::path_point> path)
{
    double worst_cost = 0.0;

    for(int i=0; i<path.size(); i++)
    {
        grid_map::Position position;
        position.x() = path[i].x - current_pose_.pose.position.x;
        position.y() = path[i].y - current_pose_.pose.position.y;

        if(map_.isInside(position))
        {
            double cost = map_.atPosition("cost_map", position);
            if(worst_cost < cost) worst_cost = cost;
        }
    }
    return worst_cost;
}

double DWA::LinearVelCost_(double linear_vel)
{
    return (linear_vel_max_ - linear_vel) / linear_vel_max_;
}

double DWA::HeadingGoalCost_(DWA::path_point terminal_point)
{
    double error_angle = atan2(target_pos_.y-terminal_point.y, target_pos_.x-terminal_point.x) - terminal_point.theta;
    return abs(error_angle) / M_PI;
}

void DWA::PublishOptimizedPath_(std::vector<DWA::path_point> opt_path)
{
    nav_msgs::Path path_points;
    ros::Time current_time = ros::Time::now();
    path_points.header.stamp = current_time;
    path_points.header.frame_id = map_frame_;

    for(int i=0; i<opt_path.size(); i++)
    {
        geometry_msgs::PoseStamped path_point;
        path_point.pose.position.x = opt_path[i].x;
        path_point.pose.position.y = opt_path[i].y;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(opt_path[i].theta);
        path_point.pose.orientation.x = quat.x;
        path_point.pose.orientation.y = quat.y;
        path_point.pose.orientation.z = quat.z;
        path_point.pose.orientation.w = quat.w;

        path_point.header.stamp = current_time;
        path_point.header.frame_id = map_frame_;
        path_points.poses.push_back(path_point);
    }

    opt_path_pub_.publish(path_points);
}

double DWA::EvaluatePath_(void)
{
    std::vector<std::vector<DWA::path_point>> paths;
    std::vector<DWA::path_point> opt_path;
    paths = DWA::CreateCandidatePaths_();

    double optimal_cost = 10.0;
    for(int i=0; i<paths.size(); i++)
    {
        double obs_cost, vel_cost, angle_cost, total_cost;
        obs_cost = DWA::ObstacleCost_(paths[i]);
        vel_cost = DWA::LinearVelCost_(paths[i].back().lin_v);
        angle_cost = DWA::HeadingGoalCost_(paths[i].back());   

        total_cost = weight_obs_ * obs_cost + weight_vel_ * vel_cost + weight_angle_ * angle_cost; 

        if(total_cost < optimal_cost) 
        {
            optimal_cost = total_cost;
            opt_path = paths[i];
        }
    }
    
    optimal_linear_vel_ = opt_path.back().lin_v;
    optimal_angular_vel_ = opt_path.back().ang_v;

    DWA::PublishOptimizedPath_(opt_path);

    return optimal_cost;
}

bool DWA::ObstacleClearanceCheck_(geometry_msgs::Point target_position)
{
    grid_map::Position position;
    position.x() = target_position.x;
    position.y() = target_position.y;
    if(map_.atPosition("cost_map", position) > obs_avoid_threshold_) return false;
    
    return true;
}

void DWA::PublishTargetMarker_(geometry_msgs::PoseStamped target_pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_marker";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = target_pose.pose.position.x;
    marker.pose.position.y = target_pose.pose.position.y;
    marker.pose.position.z = target_pose.pose.position.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;
    target_marker_pub_.publish(marker);

    return;
}

void DWA::PublishCmdVel_(void)
{
    int previous_nearest_index = 0;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(!current_pose_received_ || !waypoints_raw_received_ || !grid_map_received_) continue;

        int min_search_window = std::max(previous_nearest_index-target_search_interval_, 0);
        int max_search_window = std::min(previous_nearest_index+target_search_interval_, (int)wps_.poses.size()-1);
        
        int nearest_index;
        geometry_msgs::Point relative_pos;
        double relative_dist, nearest_relative_dist;

        relative_pos.x = wps_.poses[min_search_window].pose.position.x - current_pose_.pose.position.x;
        relative_pos.y = wps_.poses[min_search_window].pose.position.y - current_pose_.pose.position.y;
        nearest_relative_dist = sqrt(pow(relative_pos.x, 2) + pow(relative_pos.y, 2));

        for(nearest_index=min_search_window+1; nearest_index<max_search_window; nearest_index++)
        {
            relative_pos.x = wps_.poses[nearest_index].pose.position.x - current_pose_.pose.position.x;
            relative_pos.y = wps_.poses[nearest_index].pose.position.y - current_pose_.pose.position.y;
            relative_dist = sqrt(pow(relative_pos.x, 2) + pow(relative_pos.y, 2));

            if(nearest_relative_dist < relative_dist) break;
            nearest_relative_dist = relative_dist;
        }
        previous_nearest_index = nearest_index;

        int target_index;
        for(target_index=nearest_index; target_index<max_search_window; target_index++)
        {
            relative_pos.x = wps_.poses[target_index].pose.position.x - current_pose_.pose.position.x;
            relative_pos.y = wps_.poses[target_index].pose.position.y - current_pose_.pose.position.y;
            relative_dist = sqrt(pow(relative_pos.x, 2) + pow(relative_pos.y, 2));

            if(relative_dist > lookahead_dist_) 
            {
                if(map_.getLength().x()/2.0 > abs(relative_pos.x) && map_.getLength().y()/2.0 > abs(relative_pos.y))
                {
                    if(DWA::ObstacleClearanceCheck_(relative_pos)) break;
                }
                else
                {
                    break;
                }  
            }
        }
        target_pos_ = wps_.poses[target_index].pose.position;
        DWA::PublishTargetMarker_(wps_.poses[target_index]);
        
        DWA::EvaluatePath_();

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = optimal_linear_vel_;
        cmd_vel.angular.z = optimal_angular_vel_;
        twist_pub_.publish(cmd_vel);

        loop_rate.sleep();
    }
    return;
}
