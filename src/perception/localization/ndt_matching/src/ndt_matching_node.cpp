#include <ros/ros.h>
#include <ndt_matching/ndt_matching.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ndt_matching_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    NdtMatching ndt_matching(nh,pnh);
    ros::spin();
    return 0;
}
