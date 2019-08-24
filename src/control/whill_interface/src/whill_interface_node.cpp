#include <whill_interface/whill_interface.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "whill_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WhillInterface whill_interface(nh,pnh);
    ros::spin();
    return 0;
}
