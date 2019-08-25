#ifndef WHILL_INTERFACE_WHILL_INTERFACE_H_INCLUDED
#define WHILL_INTERFACE_WHILL_INTERFACE_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class WhillInterface
{
public:
    WhillInterface(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WhillInterface();
private:
    void PublishJoy_(void);
    void TwistCallback_(const geometry_msgs::Twist::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    std::string joy_topic_;
    std::string twist_topic_;

    geometry_msgs::Twist twist_msg_;
    
    ros::Publisher joy_pub_;
    ros::Subscriber twist_sub_;
};

#endif  //WHILL_INTERFACE_WHILL_INTERFACE_H_INCLUDED

