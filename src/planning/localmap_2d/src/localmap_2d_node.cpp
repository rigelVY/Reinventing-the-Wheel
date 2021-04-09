#include <localmap_2d/localmap_2d.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localmap_2d_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    LocalMap2D localmap_2d(nh,pnh);
    ros::spin();
    return 0;
}
