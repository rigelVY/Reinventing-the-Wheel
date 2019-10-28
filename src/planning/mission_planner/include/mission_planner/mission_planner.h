#ifndef MISSION_PLANNER_MISSION_PLANNER_H_INCLUDED
#define MISSION_PLANNER_MISSION_PLANNER_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

#include <rostate_machine/event_client.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class MissionPlanner
{
public:
    MissionPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~MissionPlanner();
private:
    void JoyCallback_(const sensor_msgs::Joy::ConstPtr msg);

    boost::optional<rostate_machine::Event> AutonomousStateCallback_(void);
    boost::optional<rostate_machine::Event> ManualStateCallback_(void);
    boost::optional<rostate_machine::Event> StoppingStateCallback_(void);
    boost::optional<rostate_machine::Event> MainMissionCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionAlphaCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionBravoCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionCharlieCallback_(void);
    boost::optional<rostate_machine::Event> OptionalMissionDeltaCallback_(void);

    rostate_machine::EventClient control_client_;
    rostate_machine::EventClient mission_client_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string sub_joy_topic_;

    sensor_msgs::Joy sub_joy_msg_;
    
    ros::Subscriber joy_sub_;
};

#endif  //MISSION_PLANNER_MISSION_PLANNER_H_INCLUDED

