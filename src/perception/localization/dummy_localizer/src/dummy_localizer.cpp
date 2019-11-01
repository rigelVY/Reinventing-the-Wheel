#include <dummy_localizer/dummy_localizer.h>

DummyLocalizer::DummyLocalizer(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),tf_listener_(tf_buffer_)
{
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("pose_topic", pose_topic_, "current_pose");
    current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    boost::thread publish_thread(boost::bind(&DummyLocalizer::PublishCurrentPoseStamped_, this));
}

DummyLocalizer::~DummyLocalizer()
{

}

void DummyLocalizer::PublishCurrentPoseStamped_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        geometry_msgs::TransformStamped transform_stamped;
        try 
        {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
        } 
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("Could NOT transform  %s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id = base_frame_;
        source_pose.header.stamp = ros::Time(0);
        source_pose.pose.orientation.w = 1.0;

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = map_frame_;
        target_pose.header.stamp = ros::Time(0);
        tf2::doTransform(source_pose, target_pose, transform_stamped);

        current_pose_pub_.publish(target_pose);
        loop_rate.sleep();
    }
    return;
}
