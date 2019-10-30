#include <dijkstra_planner/dijkstra_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dijkstra_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    AStarPlanner dijkstra_planner(nh,pnh);
    ros::spin();
    return 0;
}
