#include <checkpoint_manager/checkpoint_manager.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "checkpoint_manager_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle pnh("~");
    CheckpointManager checkpoint_manager(nh,pnh);
    ros::spin();
    return 0;
}
