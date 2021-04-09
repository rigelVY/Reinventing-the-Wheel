#include <waypoint_selector/waypoint_selector.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WaypointSelector waypoint_selector(nh,pnh);
    ros::spin();
    return 0;
}
