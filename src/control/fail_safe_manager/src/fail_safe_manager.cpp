#include <fail_safe_manager/fail_safe_manager.h>

FailSafeManager::FailSafeManager(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),client_(nh,pnh,"control_state_machine_node"),able_to_move_(true)
{
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("front_frame", front_frame_, "base_front");
    pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "local_grid_map");
    local_gridmap_sub_= nh_.subscribe(grid_map_topic_, 1, &FailSafeManager::LocalGridMapCallback_, this);

    FailSafeManager::GetTransformBaseToFront();

    client_.registerCallback(std::bind(&FailSafeManager::AutonomousStateCallback_, this),"autonomous_driving");
    client_.registerCallback(std::bind(&FailSafeManager::ManualStateCallback_, this),"manual_driving");
    client_.registerCallback(std::bind(&FailSafeManager::StoppingStateCallback_, this),"stopping");

    client_.run();
}

FailSafeManager::~FailSafeManager()
{

}

void FailSafeManager::GetTransformBaseToFront(void)
{
    tf::TransformListener listener;
    tf::StampedTransform tf_base_to_front;
    try
    {
        ros::Time now = ros::Time(0);
        listener.waitForTransform(base_frame_, front_frame_, now, ros::Duration(1.0));
        listener.lookupTransform(base_frame_, front_frame_, now, tf_base_to_front);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    base_front_pos_.x() = tf_base_to_front.getOrigin().x();
    base_front_pos_.y() = tf_base_to_front.getOrigin().y();
}

void FailSafeManager::LocalGridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);

    // if(map_.atPosition("cost_map", base_pos_) < 0.5 && map_.atPosition("cost_map", base_front_pos_) < 0.5) able_to_move_ = true;
    if(map_.atPosition("cost_map", base_front_pos_) < 0.5) able_to_move_ = true;
    else able_to_move_ = false;

    return;
}

boost::optional<rostate_machine::Event> FailSafeManager::AutonomousStateCallback_(void)
{
    if(!able_to_move_)
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> FailSafeManager::ManualStateCallback_(void)
{
    if(!able_to_move_)
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> FailSafeManager::StoppingStateCallback_(void)
{
    if(able_to_move_)
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "recovery_manual";
        return ret;
    }

    return boost::none;
}
