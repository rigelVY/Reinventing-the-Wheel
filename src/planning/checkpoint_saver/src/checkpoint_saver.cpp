#include <checkpoint_saver/checkpoint_saver.h>

CheckpointSaver::CheckpointSaver(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("cp_marker_topic", cp_marker_topic_, "checkpoint_saver_marker");
    pnh_.param<std::string>("checkpoints_csv", checkpoints_path_, "/tmp/saved_checkpoints.csv");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    cp_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(cp_marker_topic_, 1);

    CheckpointSaver::InitializeCheckpointsFile_();

    checkpoint_sub_ = nh_.subscribe("move_base_simple/goal", 10, &CheckpointSaver::CheckpointCallback_, this);
}

CheckpointSaver::~CheckpointSaver()
{
    ofs_.close();
}

void CheckpointSaver::InitializeCheckpointsFile_(void)
{
    ofs_.open(checkpoints_path_, std::ios::app);
    ofs_ << "label,x,y,z,yaw" << std::endl;
}

void CheckpointSaver::PublishPointsMarkerArray_(const geometry_msgs::PoseStamped pose)
{
    static visualization_msgs::MarkerArray marray;
    static int id = 0;

    // initialize marker
    static visualization_msgs::Marker marker;
    marker.id = id;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time();
    marker.frame_locked = true;

    // create saved waypoint marker
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 25.0 / 255.0;
    marker.color.g = 255.0 / 255.0;
    marker.color.b = 240.0 / 255.0;
    marker.color.a = 1.0;
    marker.ns = "saved_checkpoints";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marray.markers.push_back(marker);

    cp_marker_pub_.publish(marray);
    id++;

    return;
}

void CheckpointSaver::CheckpointCallback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::cout << "Set checkpoint." << std::endl;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        ros::Time now = ros::Time(0);
        listener.waitForTransform(map_frame_, msg->header.frame_id, now, ros::Duration(5.0));
        listener.lookupTransform(map_frame_, msg->header.frame_id, now, transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    geometry_msgs::PoseStamped cp_pose;
    cp_pose.pose.position.x = msg->pose.position.x + transform.getOrigin().x();
    cp_pose.pose.position.y = msg->pose.position.y + transform.getOrigin().y();
    cp_pose.pose.position.z = msg->pose.position.z + transform.getOrigin().z();
    cp_pose.pose.orientation = msg->pose.orientation;

    ofs_ << std::fixed << std::setprecision(4) << "," << cp_pose.pose.position.x << "," << cp_pose.pose.position.y << ","
         << cp_pose.pose.position.z << "," << tf::getYaw(cp_pose.pose.orientation) << std::endl;
    CheckpointSaver::PublishPointsMarkerArray_(cp_pose);
    
    return;
}