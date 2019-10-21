#include <dwa/dwa.h>

DWA::DWA(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");
    pnh_.param<std::string>("path_topic", path_topic_, "waypoints_raw");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "local_grid_map");
    pnh_.param<std::string>("opt_path_topic", opt_path_topic_, "opt_path_points");
    pnh_.param<double>("max_linear_vel", max_linear_vel_, 1.0);
    pnh_.param<double>("min_linear_vel", min_linear_vel_, 0.05);
    pnh_.param<double>("max_angular_vel", max_angular_vel_, 0.5);
    pnh_.param<double>("min_angular_vel", min_angular_vel_, -0.5);
    pnh_.param<double>("lookahead_distance", lookahead_dist_, 0.5);
    pnh_.param<double>("weight_obs", weight_obs_, 1.0);
    pnh_.param<double>("weight_vel", weight_vel_, 0.3);
    pnh_.param<double>("weight_angle", weight_angle_, 1.0);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 1);
    opt_path_pub_ = nh_.advertise<nav_msgs::Path>(opt_path_topic_, 1);
    path_sub_ = nh_.subscribe(path_topic_, 1, &DWA::WaypointsRawCallback_, this);
    current_pose_sub_ = nh_.subscribe(current_pose_topic_, 1, &DWA::CurrentPoseCallback_, this);
    local_gridmap_sub_= nh_.subscribe(grid_map_topic_, 1, &DWA::LocalGridMapCallback_, this);
    boost::thread publish_thread(boost::bind(&DWA::PublishCmdVel_, this));

    current_pose_received_ = 0;
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
    current_pose_received_ = 1;
    return;
}

void DWA::LocalGridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    return;
}

std::vector<DWA::path_point> DWA::CreatePath_(double linear_vel, double angular_vel, double dt, double move_time)
{
    std::vector<DWA::path_point> path;
    path.reserve(std::ceil(move_time/dt));
    DWA::path_point point;
    point.x = current_pose_.pose.position.x;
    point.y = current_pose_.pose.position.y;
    point.theta = tf::getYaw(current_pose_.pose.orientation);
    point.lin_v = linear_vel;
    point.ang_v = angular_vel;

    for(double t=0.0; t<move_time; t+=dt)
    {
        point.x += linear_vel * cos(point.theta) * t;
        point.y += linear_vel * sin(point.theta) * t;
        point.theta += angular_vel * t;
        path.emplace_back(point);
    }
    return path;
}

std::vector<std::vector<DWA::path_point>> DWA::CreateCandidatePaths_(double dt, double move_time, double resolution)
{
    std::vector<std::vector<DWA::path_point>> paths;
    for(double lin_v=min_linear_vel_; lin_v<max_linear_vel_; lin_v+=resolution)
    {
        for(double ang_v=min_angular_vel_; ang_v<max_angular_vel_; ang_v+=resolution)
        {
            paths.emplace_back(CreatePath_(lin_v, ang_v, dt, move_time));
        }
    }
    return paths;
}

double DWA::ObstacleCost_(std::vector<DWA::path_point> path)
{
    // tf::TransformListener listener;
    // tf::StampedTransform tf_map_to_lidar;
    // try
    // {
    //     ros::Time now = ros::Time(0);
    //     listener.waitForTransform(map_frame_, "lidar_link", now, ros::Duration(5.0));
    //     listener.lookupTransform(map_frame_, "lidar_link", now, tf_map_to_lidar);
    // }
    // catch (tf::TransformException& ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    // }

    // grid_map::Position base_position;
    // base_position.x() = tf_map_to_lidar.getOrigin().x();
    // base_position.y() = tf_map_to_lidar.getOrigin().y();

    double worst_cost = 0.0;

    for(int i=0; i<path.size(); i++)
    {
        grid_map::Position position;
        // position.x() = path[i].x - base_position.x();
        // position.y() = path[i].y - base_position.y();
        position.x() = path[i].x - current_pose_.pose.position.x;
        position.y() = path[i].y - current_pose_.pose.position.y;
        double cost = map_.atPosition("cost_map", position) * 2.0; // multiple 2.0 for normalized
        if(worst_cost < cost) worst_cost = cost;
    }

    // printf("worst cost; %f\n", worst_cost);
    return worst_cost;
}

double DWA::LinearVelCost_(double linear_vel)
{
    return (max_linear_vel_ - linear_vel) / max_linear_vel_;
}

double DWA::HeadingGoalCost_(double terminal_angle)
{
    return abs(terminal_angle - target_relative_angle_) / M_PI;
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

double DWA::EvaluatePath_(double dt, double move_time)
{
    std::vector<std::vector<DWA::path_point>> paths;
    std::vector<DWA::path_point> opt_path;
    double resolution = 0.1;
    paths = DWA::CreateCandidatePaths_(dt, move_time, resolution);

    double optimal_cost = 10.0;

    for(int i=0; i<paths.size(); i++)
    {
        double obs_cost, vel_cost, angle_cost, total_cost;
        obs_cost = DWA::ObstacleCost_(paths[i]);
        vel_cost = DWA::LinearVelCost_(paths[i][0].lin_v);
        angle_cost = DWA::HeadingGoalCost_(paths[i].back().theta);   

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

void DWA::PublishCmdVel_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(!current_pose_received_) continue;

        int target_index;
        geometry_msgs::Point relative_pos;
        double relative_dist, relative_angle;

        for(target_index=0; target_index<wps_.poses.size(); target_index++)
        {      
            relative_pos.x = wps_.poses[target_index].pose.position.x - current_pose_.pose.position.x;
            relative_pos.y = wps_.poses[target_index].pose.position.y - current_pose_.pose.position.y;
            relative_dist = std::sqrt(std::pow(relative_pos.x, 2) + std::pow(relative_pos.y, 2));
            relative_angle = std::atan2(relative_pos.y, relative_pos.x) - tf::getYaw(current_pose_.pose.orientation);

            if(std::abs(relative_angle) < M_PI/2)
            {
                if(relative_dist > lookahead_dist_) break;
            }
        }
        target_relative_dist_ = relative_dist;
        target_relative_angle_ = relative_angle;

        double dt = 0.02, move_time = 0.4;
        DWA::EvaluatePath_(dt, move_time);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = optimal_linear_vel_;
        cmd_vel.angular.z = optimal_angular_vel_;
        twist_pub_.publish(cmd_vel);

        loop_rate.sleep();
    }
    return;
}
