#include <checkpoint_saver/checkpoint_saver.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "checkpoint_saver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    CheckpointSaver checkpoint_saver(nh,pnh);
    ros::spin();
    return 0;
}
