#include <roomba_interface/roomba_interface.h>

RoombaInterface::RoombaInterface(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh),client_(nh,pnh,"control_state_machine_node")
{
    pnh_.param<std::string>("mode", mode_, "simulation");
    pnh_.param<std::string>("pub_sim_twist_topic", pub_sim_twist_topic_, "cmd_vel");
    pnh_.param<std::string>("pub_real_twist_topic", pub_real_twist_topic_, "cmd_vel");
    pnh_.param<std::string>("sub_twist_topic", sub_twist_topic_, "cmd_vel");
    pnh_.param<std::string>("sub_joy_topic", sub_joy_topic_, "joy");
    sim_twist_pub_ = nh_.advertise<geometry_msgs::Twist>(pub_sim_twist_topic_, 1);
    real_twist_pub_ = nh_.advertise<geometry_msgs::Twist>(pub_real_twist_topic_, 1);
    joy_sub_ = nh_.subscribe(sub_joy_topic_, 1, &RoombaInterface::JoyCallback_, this);
    twist_sub_ = nh_.subscribe(sub_twist_topic_, 1, &RoombaInterface::TwistCallback_, this);

    client_.registerCallback(std::bind(&RoombaInterface::AutonomousStateCallback_, this),"autonomous_driving");
    client_.registerCallback(std::bind(&RoombaInterface::ManualStateCallback_, this),"manual_driving");
    client_.registerCallback(std::bind(&RoombaInterface::StoppingStateCallback_, this),"stopping");

    client_.run();
}

RoombaInterface::~RoombaInterface()
{

}

void RoombaInterface::TwistCallback_(const geometry_msgs::Twist::ConstPtr msg)
{
    sub_twist_msg_ = *msg;
    return;
}

void RoombaInterface::JoyCallback_(const sensor_msgs::Joy::ConstPtr msg)
{
    sub_joy_msg_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> RoombaInterface::AutonomousStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        sim_twist_pub_.publish(sub_twist_msg_);
    }
    else if(mode_ == "real")
    {
        real_twist_pub_.publish(sub_twist_msg_);
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

    return boost::none;
}

boost::optional<rostate_machine::Event> RoombaInterface::ManualStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = sub_joy_msg_.axes[1];
        cmd_vel.angular.z = sub_joy_msg_.axes[0];
        sim_twist_pub_.publish(cmd_vel);
    }
    else if(mode_ == "real")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = sub_joy_msg_.axes[1];
        cmd_vel.angular.z = sub_joy_msg_.axes[0];
        real_twist_pub_.publish(cmd_vel);
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

    return boost::none;
}

boost::optional<rostate_machine::Event> RoombaInterface::StoppingStateCallback_(void)
{
    if(mode_ == "simulation")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = 0.0;
        cmd_vel.angular.z = 0.0;
        sim_twist_pub_.publish(cmd_vel);
    }
    else if(mode_ == "real")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x  = 0.0;
        cmd_vel.angular.z = 0.0;
        real_twist_pub_.publish(cmd_vel);
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

    return boost::none;
}
