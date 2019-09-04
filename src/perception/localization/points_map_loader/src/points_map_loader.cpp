#include <points_map_loader/points_map_loader.h>

PointsMapLoader::PointsMapLoader(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("map_path", map_path_, "/tmp/default.pcd");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("points_map_topic", points_map_topic_, "points_map");
    points_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_map_topic_, 1);
    boost::thread publish_thread(boost::bind(&PointsMapLoader::PublishPointsMap_, this));
}

PointsMapLoader::~PointsMapLoader()
{

}

void PointsMapLoader::PublishPointsMap_(void)
{
    // Loading second scan of room from new perspective
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_path_, *input_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read the PCD file \n");
        return;
    }

    sensor_msgs::PointCloud2 points_map_msg;
    pcl::toROSMsg(*input_cloud, points_map_msg);

    points_map_msg.header.stamp = ros::Time::now();
    points_map_msg.header.frame_id = map_frame_;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        points_map_pub_.publish(points_map_msg);
        loop_rate.sleep();
    }
    return;
}
