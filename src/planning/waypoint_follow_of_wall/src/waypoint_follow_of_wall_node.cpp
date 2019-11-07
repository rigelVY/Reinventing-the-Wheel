#include <waypoint_follow_of_wall/waypoint_follow_of_wall.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_follow_of_wall_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    WaypointFollowOfWall waypoint_publisher(nh, pnh);
    ros::spin();
    return 0;
} 
