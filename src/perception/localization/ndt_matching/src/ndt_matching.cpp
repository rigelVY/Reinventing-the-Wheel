#include <ndt_matching/ndt_matching.h>

NdtMatching::NdtMatching(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size_, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 1.0);
  pnh_.param<double>("max_iter", max_iter_, 30);
  pnh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("ndt_pose_topic", ndt_pose_topic_, "ndt_pose");
  pnh_.param<std::string>("map_topic", map_topic_, "points_map");
  pnh_.param<std::string>("points_topic", points_topic_, "filtered_points");
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 10);
  map_sub_ = nh_.subscribe(map_topic_, 10, &NdtMatching::MapCallback_, this);
  initialpose_sub_ = nh_.subscribe("initialpose", 10, &NdtMatching::InitialposeCallback_, this);
  filtered_points_sub_ = nh_.subscribe(points_topic_, 10, &NdtMatching::PointsCallback_, this);
  boost::thread publish_thread(boost::bind(&NdtMatching::PublishNDTPose_, this));
}

NdtMatching::~NdtMatching()
{

}

void NdtMatching::MapCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (points_map_num_ != msg->width)
  {
    std::cout << "Update points_map." << std::endl;
    points_map_num_ = msg->width;

    // Convert the data type(from sensor_msgs to pcl)
    pcl::fromROSMsg(*msg, map_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_));

    // Preliminary Settings
    ndt_.setTransformationEpsilon(trans_eps_);
    ndt_.setStepSize(step_size_);
    ndt_.setResolution(ndt_res_);
    ndt_.setMaximumIterations(max_iter_);
    
    ndt_.setInputTarget(map_ptr);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());

    map_loaded_ = 1; // map is loaded at once
  }
}

void NdtMatching::InitialposeCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  std::cout << "Set initial pose." << std::endl;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform(map_frame_, msg->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform(map_frame_, msg->header.frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  initial_pose_.position.x = msg->pose.pose.position.x + transform.getOrigin().x();
  initial_pose_.position.y = msg->pose.pose.position.y + transform.getOrigin().y();
  initial_pose_.position.z = msg->pose.pose.position.z + transform.getOrigin().z();

  double initial_pose_roll, initial_pose_pitch, initial_pose_yaw;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(initial_pose_roll, initial_pose_pitch, initial_pose_yaw);
  Eigen::AngleAxisf init_rotation_x(initial_pose_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(initial_pose_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(initial_pose_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(initial_pose_.position.x, initial_pose_.position.y, initial_pose_.position.z);
  init_guess_ = (init_translation * init_rotation_x * init_rotation_y * init_rotation_z).matrix();

  // Set initial alignment estimate found using robot odometry.
  // Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  // init_guess_ = (init_translation * init_rotation).matrix ();

  init_pos_set_ = 1;
}

void NdtMatching::PointsCallback_(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  filtered_points_ = *msg;
  return;
}

void NdtMatching::PublishNDTPose_(void)
{
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    if (map_loaded_ == 1 && init_pos_set_ == 1)
    {
      ros::Time current_scan_time = filtered_points_.header.stamp;

      pcl::PointCloud<pcl::PointXYZ> filtered_scan;
      pcl::fromROSMsg(filtered_points_, filtered_scan);
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
      
      ndt_.setInputSource(filtered_scan_ptr);

      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      ndt_.align(*output_cloud, init_guess_);

      std::cout << "Normal Distributions Transform has converged:" << ndt_.hasConverged ()
            << " score: " << ndt_.getFitnessScore () << " iteration: " << ndt_.getFinalNumIteration () << std::endl;

      Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
      t = ndt_.getFinalTransformation();

      // get rotation matrix between source clound and target cloud
      tf::Matrix3x3 rot_mat; 
      rot_mat.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                      static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                      static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
      
      double ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw;
      tf::Quaternion ndt_q;
      rot_mat.getRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw);
      ndt_q.setRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw);

      // log data
      // has_converged = ndt_.hasConverged();
      // iteration = ndt_.getFinalNumIteration();
      // fitness_score = ndt_.getFitnessScore();
      // trans_probability = ndt_.getTransformationProbability();

      geometry_msgs::Pose ndt_pose;
      ndt_pose.position.x = t(0, 3);
      ndt_pose.position.y = t(1, 3);
      ndt_pose.position.z = t(2, 3);
      ndt_pose.orientation.x = ndt_q.x();
      ndt_pose.orientation.y = ndt_q.y();
      ndt_pose.orientation.z = ndt_q.z();
      ndt_pose.orientation.w = ndt_q.w();
      
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      poseMsgToTF(ndt_pose, transform);
      br.sendTransform(tf::StampedTransform(transform, current_scan_time, map_frame_, lidar_frame_));

      geometry_msgs::PoseStamped ndt_pose_msg;
      ndt_pose_msg.header.frame_id = map_frame_;
      ndt_pose_msg.header.stamp = current_scan_time;
      ndt_pose_msg.pose = ndt_pose;

      ndt_pose_pub_.publish(ndt_pose_msg);
    }
    loop_rate.sleep();
  }
  return;
}
