#ifndef ROOMBA_INTERFACE_ROOMBA_INTERFACE_H_INCLUDED
#define ROOMBA_INTERFACE_ROOMBA_INTERFACE_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <rostate_machine/event_client.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>


class RoombaInterface
{
public:
    RoombaInterface(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~RoombaInterface();
private:
    void TwistCallback_(const geometry_msgs::Twist::ConstPtr msg);
    void JoyCallback_(const sensor_msgs::Joy::ConstPtr msg);

    boost::optional<rostate_machine::Event> AutonomousStateCallback_(void);
    boost::optional<rostate_machine::Event> ManualStateCallback_(void);
    boost::optional<rostate_machine::Event> StoppingStateCallback_(void);

    rostate_machine::EventClient client_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string mode_;
    std::string pub_sim_twist_topic_;
    std::string pub_real_twist_topic_;
    std::string sub_joy_topic_;
    std::string sub_twist_topic_;

    geometry_msgs::Twist sub_twist_msg_;
    sensor_msgs::Joy sub_joy_msg_;
    
    ros::Publisher sim_twist_pub_;
    ros::Publisher real_twist_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber twist_sub_;
};

#endif  //ROOMBA_INTERFACE_ROOMBA_INTERFACE_H_INCLUDED

