#ifndef FAIL_SAFE_MANAGER_FAIL_SAFE_MANAGER_H_INCLUDED
#define FAIL_SAFE_MANAGER_FAIL_SAFE_MANAGER_H_INCLUDED

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <rostate_machine/event_client.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>


class FailSafeManager
{
public:
    FailSafeManager(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~FailSafeManager();
private:
    void LocalGridMapCallback_(const grid_map_msgs::GridMap::ConstPtr msg);
    void GetTransformBaseToFront(void);

    boost::optional<rostate_machine::Event> AutonomousStateCallback_(void);
    boost::optional<rostate_machine::Event> ManualStateCallback_(void);
    boost::optional<rostate_machine::Event> StoppingStateCallback_(void);

    rostate_machine::EventClient client_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    bool able_to_move_;

    grid_map::GridMap map_;
    grid_map::Position base_pos_, base_front_pos_;
    
    std::string base_frame_;
    std::string front_frame_;
    std::string grid_map_topic_;
    
    ros::Subscriber local_gridmap_sub_;
};

#endif  //FAIL_SAFE_MANAGER_FAIL_SAFE_MANAGER_H_INCLUDED

