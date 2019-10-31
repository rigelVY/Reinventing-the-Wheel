#include <waypoint_selector/waypoint_selector.h>

WaypointSelector::WaypointSelector(ros::NodeHandle nh,ros::NodeHandle pnh) 
: nh_(nh),pnh_(pnh),mission_client_(nh,pnh,"mission_state_machine_node")
{
    pnh_.param<std::string>("main_mission_path_topic", main_mission_path_topic_, "waypoints_raw");
    pnh_.param<std::string>("mission_a_path_topic", mission_a_path_topic_, "waypoints_raw");
    pnh_.param<std::string>("mission_b_path_topic", mission_b_path_topic_, "waypoints_raw");
    pnh_.param<std::string>("mission_c_path_topic", mission_c_path_topic_, "waypoints_raw");
    pnh_.param<std::string>("mission_d_path_topic", mission_d_path_topic_, "waypoints_raw");
    pnh_.param<std::string>("pub_path_topic", pub_path_topic_, "waypoint_selector/path");
    main_path_sub_ = nh_.subscribe(main_mission_path_topic_, 10, &WaypointSelector::MainMissionWaypointsCallback_, this);
    mission_a_path_sub_ = nh_.subscribe(mission_a_path_topic_, 10, &WaypointSelector::MissionAlphaWaypointsCallback_, this);
    mission_b_path_sub_ = nh_.subscribe(mission_b_path_topic_, 10, &WaypointSelector::MissionBravoWaypointsCallback_, this);
    mission_c_path_sub_ = nh_.subscribe(mission_c_path_topic_, 10, &WaypointSelector::MissionCharlieWaypointsCallback_, this);
    mission_d_path_sub_ = nh_.subscribe(mission_d_path_topic_, 10, &WaypointSelector::MissionDeltaWaypointsCallback_, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(pub_path_topic_, 1);

    mission_client_.registerCallback(std::bind(&WaypointSelector::MainMissionCallback_, this),"main_mission");
    mission_client_.registerCallback(std::bind(&WaypointSelector::OptionalMissionAlphaCallback_, this),"optional_mission_A");
    mission_client_.registerCallback(std::bind(&WaypointSelector::OptionalMissionBravoCallback_, this),"optional_mission_B");
    mission_client_.registerCallback(std::bind(&WaypointSelector::OptionalMissionCharlieCallback_, this),"optional_mission_C");
    mission_client_.registerCallback(std::bind(&WaypointSelector::OptionalMissionDeltaCallback_, this),"optional_mission_D");
    mission_client_.run();
}

WaypointSelector::~WaypointSelector()
{

}

void WaypointSelector::MainMissionWaypointsCallback_(const nav_msgs::Path::ConstPtr msg)
{
    main_mission_wps_ = *msg;
    return;
}

void WaypointSelector::MissionAlphaWaypointsCallback_(const nav_msgs::Path::ConstPtr msg)
{
    mission_a_wps_ = *msg;
    return;
}

void WaypointSelector::MissionBravoWaypointsCallback_(const nav_msgs::Path::ConstPtr msg)
{
    mission_b_wps_ = *msg;
    return;
}

void WaypointSelector::MissionCharlieWaypointsCallback_(const nav_msgs::Path::ConstPtr msg)
{
    mission_c_wps_ = *msg;
    return;
}

void WaypointSelector::MissionDeltaWaypointsCallback_(const nav_msgs::Path::ConstPtr msg)
{
    mission_d_wps_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> WaypointSelector::MainMissionCallback_(void)
{
    path_pub_.publish(main_mission_wps_);
    return boost::none;
}

boost::optional<rostate_machine::Event> WaypointSelector::OptionalMissionAlphaCallback_(void)
{
    path_pub_.publish(mission_a_wps_);
    return boost::none;
}

boost::optional<rostate_machine::Event> WaypointSelector::OptionalMissionBravoCallback_(void)
{
    path_pub_.publish(mission_b_wps_);
    return boost::none;
}

boost::optional<rostate_machine::Event> WaypointSelector::OptionalMissionCharlieCallback_(void)
{
    path_pub_.publish(mission_c_wps_);
    return boost::none;
}

boost::optional<rostate_machine::Event> WaypointSelector::OptionalMissionDeltaCallback_(void)
{  
    path_pub_.publish(mission_d_wps_);
    return boost::none;
}
