#include <astar_planner/astar_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astar_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    AStarPlanner astar_planner(nh,pnh);
    ros::spin();
    return 0;
}
