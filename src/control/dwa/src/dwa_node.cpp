#include <dwa/dwa.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dwa_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DWA dwa(nh,pnh);
    ros::spin();
    return 0;
}
