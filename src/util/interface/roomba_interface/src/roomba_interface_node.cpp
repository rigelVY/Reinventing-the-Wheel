#include <roomba_interface/roomba_interface.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roomba_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RoombaInterface roomba_interface(nh,pnh);
    ros::spin();
    return 0;
}
