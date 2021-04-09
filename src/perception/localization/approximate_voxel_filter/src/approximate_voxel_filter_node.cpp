#include <approximate_voxel_filter/approximate_voxel_filter.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "approximate_voxel_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ApproximateVoxelFilter approximate_voxel_filter(nh,pnh);
    ros::spin();
    return 0;
}
