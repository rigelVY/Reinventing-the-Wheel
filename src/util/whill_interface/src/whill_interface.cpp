#include <whill_interface/whill_interface.h>

WhillInterface::WhillInterface(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),client_(nh,pnh,"control_state_machine_node")
{
    pnh_.param<std::string>("mode", mode_, "simulation");
    pnh_.param<std::string>("pub_twist_topic", pub_twist_topic_, "cmd_vel");
    pnh_.param<std::string>("pub_joy_topic", pub_joy_topic_, "joy");
    pnh_.param<std::string>("sub_twist_topic", sub_twist_topic_, "cmd_vel");
    pnh_.param<std::string>("sub_joy_topic", sub_joy_topic_, "joy");
    joy_pub_ = nh_.advertise<sensor_msgs::Joy>(pub_joy_topic_, 1);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(pub_twist_topic_, 1);
    joy_sub_ = nh_.subscribe(sub_joy_topic_, 1, &WhillInterface::JoyCallback_, this);
    twist_sub_ = nh_.subscribe(sub_twist_topic_, 1, &WhillInterface::TwistCallback_, this);

    client_.registerCallback(std::bind(&WhillInterface::AutonomousStateCallback_, this),"autonomous_driving");
    client_.registerCallback(std::bind(&WhillInterface::ManualStateCallback_, this),"manual_driving");
    client_.registerCallback(std::bind(&WhillInterface::StoppingStateCallback_, this),"stopping");

    client_.run();
}

WhillInterface::~WhillInterface()
{

}

void WhillInterface::TwistCallback_(const geometry_msgs::Twist::ConstPtr msg)
{
    sub_twist_msg_ = *msg;
    return;
}

void WhillInterface::JoyCallback_(const sensor_msgs::Joy::ConstPtr msg)
{
    sub_joy_msg_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> WhillInterface::AutonomousStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        twist_pub_.publish(sub_twist_msg_);
        // printf("sub_twist_msg_.linear.x: %f\n", sub_twist_msg_.linear.x);
    }
    else if(mode_ == "real")
    {
        sensor_msgs::Joy joy_msg;
        joy_msg.header.stamp=ros::Time::now();
        joy_msg.header.frame_id="";
        joy_msg.axes.resize(2);
        joy_msg.axes[1] = sub_twist_msg_.linear.x;
        joy_msg.axes[0] = sub_twist_msg_.angular.z;
        joy_pub_.publish(joy_msg);
    }

    if(sub_joy_msg_.buttons[0])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }
    else if(sub_joy_msg_.buttons[5])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "manual_override";
        return ret;
    }

    ROS_INFO("autonomous driving now!");
    return boost::none;
}

boost::optional<rostate_machine::Event> WhillInterface::ManualStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = sub_joy_msg_.axes[1];
        cmd_vel.angular.z = sub_joy_msg_.axes[0];
        twist_pub_.publish(cmd_vel);
    }
    else if(mode_ == "real")
    {
        joy_pub_.publish(sub_joy_msg_);
    }

    if(sub_joy_msg_.buttons[0])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "stop";
        return ret;
    }
    else if(sub_joy_msg_.buttons[4])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "system_bringup";
        return ret;
    }

    ROS_INFO("manual driving now!");
    return boost::none;
}

boost::optional<rostate_machine::Event> WhillInterface::StoppingStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = 0.0;
        cmd_vel.angular.z = 0.0;
        twist_pub_.publish(cmd_vel);
    }
    else if(mode_ == "real")
    {
        sensor_msgs::Joy joy_msg;
        joy_msg.header.stamp=ros::Time::now();
        joy_msg.header.frame_id="";
        joy_msg.axes.resize(2);
        joy_msg.axes[1] = 0.0;
        joy_msg.axes[0] = 0.0;
        joy_pub_.publish(joy_msg);
    }

    if(sub_joy_msg_.buttons[4])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "recovery_autonomous";
        return ret;
    }
    else if(sub_joy_msg_.buttons[5])
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "recovery_manual";
        return ret;
    }

    ROS_INFO("stopping now!");
    return boost::none;
}