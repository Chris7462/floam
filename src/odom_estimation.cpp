// ros header
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// c++ header
#include <chrono>
#include <functional>
#include <stdexcept>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/odom_estimation.hpp"


namespace floam
{

OdomEstimation::OdomEstimation()
: Node("odom_estimation_node"), lidar_path_{}, processing_in_progress_(false),
  is_odom_inited_(false), total_time_(0.0), total_frame_(0)
{
  initialize_parameters();
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "Odom estimation node initialized successfully");
}

OdomEstimation::~OdomEstimation()
{
  RCLCPP_INFO(get_logger(), "Odom estimation node shutting down");
}

void OdomEstimation::initialize_parameters()
{
  // declare and load parameters directly into lidar_param_
  lidar_param_.scan_period = declare_parameter<double>("scan_period", 0.1);
  lidar_param_.vertical_angle = declare_parameter<double>("vertical_angle", 2.0);
  lidar_param_.num_scan_lines = declare_parameter<int>("num_scan_lines", 64);
  lidar_param_.max_distance = declare_parameter<double>("max_dist", 90.0);
  lidar_param_.min_distance = declare_parameter<double>("min_dist", 2.0);
  const double map_resolution = declare_parameter<double>("map_resolution", 0.4);
  queue_size_ = declare_parameter<int>("queue_size", 10);
  max_processing_queue_size_ =
    static_cast<size_t>(declare_parameter<int>("max_processing_queue_size", 3));
  processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
  input_edge_topic_ = declare_parameter<std::string>("input_edge_topic", "lidar_cloud_edge");
  input_surf_topic_ = declare_parameter<std::string>("input_surf_topic", "lidar_cloud_surf");
  output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "odom");
  output_path_topic_ = declare_parameter<std::string>("output_path_topic", "odom_path");

  // validate processing frequency to avoid division by zero in timer creation
  if (processing_frequency_ <= 0) {
    throw std::runtime_error("Invalid processing frequency: " +
      std::to_string(processing_frequency_));
  }

  odom_estimation_.init(map_resolution);

  RCLCPP_INFO(get_logger(),
    "Parameters initialized - num_scan_lines: %d, scan_period: %.2f, max_dist: %.1f, min_dist: %.1f, "
    "map_resolution: %.2f, processing_frequency: %.1f Hz, max_processing_queue_size: %zu",
    lidar_param_.num_scan_lines, lidar_param_.scan_period, lidar_param_.max_distance,
    lidar_param_.min_distance, map_resolution, processing_frequency_, max_processing_queue_size_);
}

void OdomEstimation::initialize_ros_components()
{
  // configure QoS profile for lidar point cloud transport
  rclcpp::QoS lidar_qos(queue_size_);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // create reentrant callback group for parallel execution
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // create message filters subscribers
  sub_edge_lidar_cloud_.subscribe(this, input_edge_topic_, lidar_qos.get_rmw_qos_profile());
  sub_surf_lidar_cloud_.subscribe(this, input_surf_topic_, lidar_qos.get_rmw_qos_profile());

  // create exact time synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(queue_size_), sub_edge_lidar_cloud_, sub_surf_lidar_cloud_);
  sync_->registerCallback(
    std::bind(&OdomEstimation::lidar_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  pub_lidar_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic_, lidar_qos);
  pub_lidar_path_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic_, lidar_qos);

  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&OdomEstimation::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "ROS components initialized");
  RCLCPP_INFO(get_logger(), "Input edge: %s, surf: %s, Output odom: %s, path: %s",
    input_edge_topic_.c_str(), input_surf_topic_.c_str(),
    output_odom_topic_.c_str(), output_path_topic_.c_str());
}

void OdomEstimation::lidar_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr surf_msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_lock_);

    if (point_cloud_buf_.size() >= max_processing_queue_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest point cloud pair (queue size: %ld)",
        point_cloud_buf_.size());
      point_cloud_buf_.pop();
    }

    point_cloud_buf_.push({edge_msg, surf_msg});

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in lidar callback: %s", e.what());
  }
}

void OdomEstimation::timer_callback()
{
  // skip if already processing — odom_estimation_ is not thread-safe
  if (processing_in_progress_.load()) {
    return;
  }

  PointCloudPair msg_pair;

  {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    if (point_cloud_buf_.empty()) {
      return;
    }
    msg_pair = point_cloud_buf_.front();
    point_cloud_buf_.pop();
  }

  processing_in_progress_.store(true);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg_pair.first, *pointcloud_edge_in);
  pcl::fromROSMsg(*msg_pair.second, *pointcloud_surf_in);
  rclcpp::Time pointcloud_time = msg_pair.first->header.stamp;

  try {
    process_odom(pointcloud_edge_in, pointcloud_surf_in);

    if (pub_lidar_odometry_->get_subscription_count() > 0 ||
      pub_lidar_path_->get_subscription_count() > 0)
    {
      publish_odom_result(pointcloud_time);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during odom estimation: %s", e.what());
  }

  processing_in_progress_.store(false);
}

void OdomEstimation::process_odom(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in)
{
  if (is_odom_inited_ == false) {
    odom_estimation_.init_map_with_points(pointcloud_edge_in, pointcloud_surf_in);
    is_odom_inited_ = true;
    RCLCPP_INFO(get_logger(), "odom inited");
  } else {
    auto start = std::chrono::system_clock::now();
    odom_estimation_.update_points_to_map(pointcloud_edge_in, pointcloud_surf_in);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = end - start;
    total_frame_++;
    total_time_ += elapsed_seconds.count() * 1000;
    // RCLCPP_INFO(get_logger(), "average odom estimation time %f ms", total_time_ / total_frame_);
  }
}

void OdomEstimation::publish_odom_result(const rclcpp::Time & pointcloud_time)
{
  // publish TF
  geometry_msgs::msg::TransformStamped t = tf2::eigenToTransform(odom_estimation_.odom_);
  t.header.stamp = pointcloud_time;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  br_->sendTransform(t);

  // publish odometry
  nav_msgs::msg::Odometry lidar_odometry;
  lidar_odometry.header.stamp = pointcloud_time;
  lidar_odometry.header.frame_id = "map";
  lidar_odometry.child_frame_id = "base_link";
  lidar_odometry.pose.pose = tf2::toMsg(odom_estimation_.odom_);
  pub_lidar_odometry_->publish(lidar_odometry);

  // publish path
  geometry_msgs::msg::PoseStamped lidar_pose;
  lidar_pose.header = lidar_odometry.header;
  lidar_pose.pose = lidar_odometry.pose.pose;
  lidar_path_.header = lidar_odometry.header;
  lidar_path_.poses.push_back(lidar_pose);
  pub_lidar_path_->publish(lidar_path_);
}

}  // namespace floam
