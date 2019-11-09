#include <fail_safe_manager/fail_safe_manager.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fail_safe_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    FailSafeManager fail_safe_manager(nh,pnh);
    ros::spin();
    return 0;
}
