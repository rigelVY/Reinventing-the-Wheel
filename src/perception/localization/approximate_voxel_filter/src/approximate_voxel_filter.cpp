#include <approximate_voxel_filter/approximate_voxel_filter.h>

ApproximateVoxelFilter::ApproximateVoxelFilter(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.2);
    pnh_.param<std::string>("filtered_points_topic", filtered_points_topic_, "filtered_points");
    pnh_.param<std::string>("points_topic", points_topic_, "points_raw");
    filtered_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 1);
    points_sub_ = nh_.subscribe(points_topic_, 1, &ApproximateVoxelFilter::PointsCallback_, this);
    boost::thread publish_thread(boost::bind(&ApproximateVoxelFilter::PublishFilteredPoints_, this));
}

ApproximateVoxelFilter::~ApproximateVoxelFilter()
{

}

void ApproximateVoxelFilter::PointsCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    points_raw_ = *msg;
    pcl::fromROSMsg(points_raw_, input_points_);
    return;
}

void ApproximateVoxelFilter::PublishFilteredPoints_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>(input_points_));
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 filtered_msg;

        // if voxel_leaf_size_ < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
        if (voxel_leaf_size_ >= 0.1)
        {
            // Downsampling the velodyne scan using VoxelGrid filter
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            approximate_voxel_filter.setInputCloud(input_cloud);
            approximate_voxel_filter.filter(*filtered_cloud);
            pcl::toROSMsg(*filtered_cloud, filtered_msg);
        }
        else
        {
            pcl::toROSMsg(*input_cloud, filtered_msg);
        }

        filtered_msg.header = points_raw_.header;
        filtered_points_pub_.publish(filtered_msg);
        loop_rate.sleep();
    }
    return;
}
