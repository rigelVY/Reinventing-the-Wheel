#include <waypoint_publisher/waypoint_publisher.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WaypointPublisher waypoint_publisher(nh, pnh);
    ros::spin();
    return 0;
}
