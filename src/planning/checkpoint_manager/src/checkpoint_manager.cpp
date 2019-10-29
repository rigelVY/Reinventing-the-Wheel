#include <checkpoint_manager/checkpoint_manager.h>

CheckpointManager::CheckpointManager(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("checkpoints_csv", checkpoints_path_, "/tmp/checkpoints.csv");
    pnh_.param<std::string>("state_array_topic", state_array_topic_, "checkpoint_manager/state");
    pnh_.param<std::string>("pict_marker_topic", pict_marker_topic_, "checkpoint_manager/pict_marker");
    pnh_.param<std::string>("cirlce_marker_topic", circle_marker_topic_, "checkpoint_manager/circle_marker");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<std::string>("reset_topic", reset_topic_, "checkpoint_manager/reset");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<double>("boundary_distance", boundary_distance_, 0.5);
    cp_states_pub_ = nh_.advertise<checkpoint_msgs::StateArray>(state_array_topic_, 10);
    pict_markers_pub_ = nh_.advertise<jsk_rviz_plugins::PictogramArray>(pict_marker_topic_, 10);
    circle_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(circle_marker_topic_, 10);
    current_pose_sub_ = nh_.subscribe(current_pose_topic_, 10, &CheckpointManager::CurrentPoseCallback_, this);
    reset_sub_ = nh_.subscribe(reset_topic_, 10, &CheckpointManager::ResetMsgCallback_, this);
    boost::thread publish_thread(boost::bind(&CheckpointManager::PublishCheckpointArray_, this));

    CheckpointManager::InitializeCheckpointArray_();
}

CheckpointManager::~CheckpointManager()
{

}

void CheckpointManager::InitializeCheckpointArray_(void)
{
    current_time_ = ros::Time::now();
    cp_states_.header.stamp = current_time_;
    cp_states_.header.frame_id = map_frame_;

    std::ifstream ifs(checkpoints_path_);

    if(!ifs)
    {
        ROS_ERROR("Error! File can not be opened");
        return;
    }

    std::string line;
    std::getline(ifs, line);  // Remove first line

    while(std::getline(ifs, line))
    {
        checkpoint_msgs::State state;
        CheckpointManager::LoadCheckpoint_(line, &state);
        cp_states_.states.push_back(state);
    }
    return;
}

void CheckpointManager::LoadCheckpoint_(const std::string& line, checkpoint_msgs::State* state)
{
    std::vector<std::string> columns;
    std::string column;
    std::istringstream stream(line);

    while (getline(stream, column, ','))
    {
        columns.push_back(column);
    }

    state->label = columns[0];
    state->position.x = std::stod(columns[1]);
    state->position.y = std::stod(columns[2]);
    state->position.z = std::stod(columns[3]);
    state->state = checkpoint_msgs::State::UNPASSED;
    
    state->header.stamp = current_time_;
    state->header.frame_id = map_frame_;

    return;
}

void CheckpointManager::CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose_ = *msg;

    for(int i=0; i<cp_states_.states.size(); i++)
    {
        double dist = sqrt(pow(cp_states_.states[i].position.x-current_pose_.pose.position.x, 2) 
                         + pow(cp_states_.states[i].position.y-current_pose_.pose.position.y, 2) 
                         + pow(cp_states_.states[i].position.z-current_pose_.pose.position.z, 2));

        if(dist <= boundary_distance_)
        {
            if(cp_states_.states[i].state == checkpoint_msgs::State::UNPASSED) cp_states_.states[i].state = checkpoint_msgs::State::PASSED_NOW;
            else cp_states_.states[i].state = checkpoint_msgs::State::PASSED;
        }
    }

    current_time_ = ros::Time::now();
    CheckpointManager::PublishCheckpointMarker_();
    cp_states_.header.stamp = current_time_;
    cp_states_pub_.publish(cp_states_);

    return;
}

void CheckpointManager::ResetMsgCallback_(const checkpoint_msgs::ResetMsg::ConstPtr msg)
{
    checkpoint_msgs::ResetMsg reset_msg = *msg;

    for(int i=0; i<reset_msg.labels.size(); i++)
    {
        for(int j=0; j<cp_states_.states.size(); j++)
        {
            if(cp_states_.states[j].label == reset_msg.labels[i])
            {
                cp_states_.states[j].state = checkpoint_msgs::State::UNPASSED;
                break;
            }
        }
    }

    current_time_ = ros::Time::now();
    CheckpointManager::PublishCheckpointMarker_();
    cp_states_.header.stamp = current_time_;
    cp_states_pub_.publish(cp_states_);

    return;
}

void CheckpointManager::PublishCheckpointMarker_(void)
{
    jsk_rviz_plugins::PictogramArray pict_markers;
    pict_markers.header.stamp = current_time_;
    pict_markers.header.frame_id = map_frame_;

    jsk_rviz_plugins::Pictogram pict;
    pict.header.stamp = current_time_;
    pict.header.frame_id = map_frame_;
    pict.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
    pict.character = "fa-angle-double-down";
    pict.size = 2.0;
    pict.action = jsk_rviz_plugins::Pictogram::ADD;
    pict.pose.orientation.x = 0.0;
    pict.pose.orientation.y = 1.0 * sin(-M_PI/4.0);
    pict.pose.orientation.z = 0.0;
    pict.pose.orientation.w = cos(-M_PI/4.0);

    visualization_msgs::MarkerArray circle_markers;
    // static int id = 0;
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.stamp = current_time_;
    marker.header.frame_id = map_frame_;
    marker.scale.x = boundary_distance_*2.0;
    marker.scale.y = boundary_distance_*2.0;
    marker.scale.z = 0.02;
    marker.ns = "checkpoint_circle";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    for(int i=0; i<cp_states_.states.size(); i++)
    {
        pict.pose.position = cp_states_.states[i].position;
        marker.pose.position = cp_states_.states[i].position;

        if(cp_states_.states[i].state == checkpoint_msgs::State::UNPASSED)
        {
            pict.color.r = 25.0 / 255.0;
            pict.color.g = 255.0 / 255.0;
            pict.color.b = 240.0 / 255.0;
            pict.color.a = 1.0;

            marker.color.r = 25.0 / 255.0;
            marker.color.g = 255.0 / 255.0;
            marker.color.b = 240.0 / 255.0;
            marker.color.a = 0.5;
        }
        else
        {
            pict.color.r = 255.0 / 255.0;
            pict.color.g = 0.0 / 255.0;
            pict.color.b = 0.0 / 255.0;
            pict.color.a = 1.0;

            marker.color.r = 255.0 / 255.0;
            marker.color.g = 0.0 / 255.0;
            marker.color.b = 0.0 / 255.0;
            marker.color.a = 0.5;
        }
        
        pict_markers.pictograms.push_back(pict);
        circle_markers.markers.push_back(marker);
        marker.id++;
    }

    pict_markers_pub_.publish(pict_markers);
    circle_markers_pub_.publish(circle_markers);

    return;
}

void CheckpointManager::PublishCheckpointArray_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        current_time_ = ros::Time::now();
        CheckpointManager::PublishCheckpointMarker_();
        cp_states_.header.stamp = current_time_;
        cp_states_pub_.publish(cp_states_);

        loop_rate.sleep();
    }
    return;
}
