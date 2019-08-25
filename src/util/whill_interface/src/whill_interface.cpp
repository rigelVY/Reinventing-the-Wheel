#include <whill_interface/whill_interface.h>

WhillInterface::WhillInterface(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("twist_topic", twist_topic_, "cmd_vel");
    pnh_.param<std::string>("joy_topic", joy_topic_, "joy");
    joy_pub_ = nh_.advertise<sensor_msgs::Joy>(joy_topic_, 1);
    twist_sub_ = nh_.subscribe(twist_topic_, 1, &WhillInterface::TwistCallback_, this);
    boost::thread publish_thread(boost::bind(&WhillInterface::PublishJoy_, this));
}

WhillInterface::~WhillInterface()
{

}

void WhillInterface::TwistCallback_(const geometry_msgs::Twist::ConstPtr msg)
{
    twist_msg_ = *msg;
    return;
}

void WhillInterface::PublishJoy_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        sensor_msgs::Joy joy_msg;
        joy_msg.header.stamp=ros::Time::now();
        joy_msg.header.frame_id="";
        joy_msg.axes.resize(2);
        joy_msg.axes[1] = twist_msg_.linear.x;
        joy_msg.axes[0] = twist_msg_.angular.z;
        joy_pub_.publish(joy_msg);

        loop_rate.sleep();
    }
    return;
}
