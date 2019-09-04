#include <points_map_loader/points_map_loader.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "points_map_loader_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PointsMapLoader points_map_loader(nh,pnh);
    ros::spin();
    return 0;
}
