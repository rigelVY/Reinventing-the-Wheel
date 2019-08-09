#include <pure_pursuit/pure_pursuit.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PurePursuit pure_pursuit(nh,pnh);
    ros::spin();
    return 0;
}
