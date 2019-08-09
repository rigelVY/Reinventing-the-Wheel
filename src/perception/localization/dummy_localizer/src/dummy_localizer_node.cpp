#include <dummy_localizer/dummy_localizer.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dummy_localizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DummyLocalizer dummy_localizer(nh,pnh);
    ros::spin();
    return 0;
}
