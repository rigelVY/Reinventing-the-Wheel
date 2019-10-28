#include <mission_planner/mission_planner.h>

MissionPlanner::MissionPlanner(ros::NodeHandle nh,ros::NodeHandle pnh) 
: nh_(nh),pnh_(pnh),control_client_(nh,pnh,"control_state_machine_node"), mission_client_(nh,pnh,"mission_state_machine_node")
{
    pnh_.param<std::string>("sub_joy_topic", sub_joy_topic_, "joy");
    joy_sub_ = nh_.subscribe(sub_joy_topic_, 1, &MissionPlanner::JoyCallback_, this);

    control_client_.registerCallback(std::bind(&MissionPlanner::AutonomousStateCallback_, this),"autonomous_driving");
    control_client_.registerCallback(std::bind(&MissionPlanner::ManualStateCallback_, this),"manual_driving");
    control_client_.registerCallback(std::bind(&MissionPlanner::StoppingStateCallback_, this),"stopping");

    mission_client_.registerCallback(std::bind(&MissionPlanner::MainMissionCallback_, this),"main_mission");
    mission_client_.registerCallback(std::bind(&MissionPlanner::OptionalMissionAlphaCallback_, this),"optional_mission_A");
    mission_client_.registerCallback(std::bind(&MissionPlanner::OptionalMissionBravoCallback_, this),"optional_mission_B");
    mission_client_.registerCallback(std::bind(&MissionPlanner::OptionalMissionCharlieCallback_, this),"optional_mission_C");
    mission_client_.registerCallback(std::bind(&MissionPlanner::OptionalMissionDeltaCallback_, this),"optional_mission_D");

    control_client_.run();
    mission_client_.run();
}

MissionPlanner::~MissionPlanner()
{

}

void MissionPlanner::JoyCallback_(const sensor_msgs::Joy::ConstPtr msg)
{
    sub_joy_msg_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> MissionPlanner::AutonomousStateCallback_(void)
{
    if(sub_joy_msg_.buttons[4])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::ManualStateCallback_(void)
{
    if(sub_joy_msg_.buttons[5])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::StoppingStateCallback_(void)
{
    if(sub_joy_msg_.buttons[6])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "recovery_autonomous";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::MainMissionCallback_(void)
{
    if(sub_joy_msg_.buttons[7])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "start_mission_C";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::OptionalMissionAlphaCallback_(void)
{

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::OptionalMissionBravoCallback_(void)
{

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::OptionalMissionCharlieCallback_(void)
{
    if(sub_joy_msg_.buttons[8])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "back_to_main_mission";
        return ret;
    }

    return boost::none;
}

boost::optional<rostate_machine::Event> MissionPlanner::OptionalMissionDeltaCallback_(void)
{

    return boost::none;
}