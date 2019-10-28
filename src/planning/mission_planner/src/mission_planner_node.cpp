#include <mission_planner/mission_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mission_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    MissionPlanner mission_planner(nh,pnh);
    ros::spin();
    return 0;
}
