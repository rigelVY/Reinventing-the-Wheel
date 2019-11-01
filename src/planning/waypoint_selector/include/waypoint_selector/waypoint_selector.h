#ifndef WAYPOINT_SELECTOR_WAYPOINT_SELECTOR_H_INCLUDED
#define WAYPOINT_SELECTOR_WAYPOINT_SELECTOR_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <rostate_machine/event_client.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class WaypointSelector
{
public:
    WaypointSelector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WaypointSelector();
private:
    void MainMissionWaypointsCallback_(const nav_msgs::Path::ConstPtr msg);
    void MissionAlphaWaypointsCallback_(const nav_msgs::Path::ConstPtr msg);
    void MissionBravoWaypointsCallback_(const nav_msgs::Path::ConstPtr msg);
    void MissionCharlieWaypointsCallback_(const nav_msgs::Path::ConstPtr msg);
    void MissionDeltaWaypointsCallback_(const nav_msgs::Path::ConstPtr msg);

    boost::optional<rostate_machine::Event> MainMissionCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionAlphaCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionBravoCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionCharlieCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionDeltaCallback_(void);

    rostate_machine::EventClient mission_client_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string main_mission_path_topic_;
    std::string mission_a_path_topic_;
    std::string mission_b_path_topic_;
    std::string mission_c_path_topic_;
    std::string mission_d_path_topic_;
    std::string pub_path_topic_;

    nav_msgs::Path main_mission_wps_;
    nav_msgs::Path mission_a_wps_;
    nav_msgs::Path mission_b_wps_;
    nav_msgs::Path mission_c_wps_;
    nav_msgs::Path mission_d_wps_;

    ros::Subscriber main_path_sub_;
    ros::Subscriber mission_a_path_sub_;
    ros::Subscriber mission_b_path_sub_;
    ros::Subscriber mission_c_path_sub_;
    ros::Subscriber mission_d_path_sub_;
    ros::Publisher path_pub_;
};

#endif  //WAYPOINT_SELECTOR_WAYPOINT_SELECTOR_H_INCLUDED

