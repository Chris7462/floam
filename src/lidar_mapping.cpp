// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <chrono>
#include <functional>
#include <stdexcept>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/lidar_mapping.hpp"


namespace floam
{

LidarMapping::LidarMapping()
  : Node("lidar_mapping_node"), processing_in_progress_(false)
{
  initialize_parameters();
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "Lidar mapping node initialized successfully");
}

LidarMapping::~LidarMapping()
{
  RCLCPP_INFO(get_logger(), "Lidar mapping node shutting down");
}

void LidarMapping::initialize_parameters()
{
  // declare and load parameters directly into lidar_param_
  lidar_param_.scan_period = declare_parameter<double>("scan_period", 0.1);
  lidar_param_.vertical_angle = declare_parameter<double>("vertical_angle", 2.0);
  lidar_param_.num_scan_lines = declare_parameter<int>("num_scan_lines", 64);
  lidar_param_.max_distance = declare_parameter<double>("max_dist", 90.0);
  lidar_param_.min_distance = declare_parameter<double>("min_dist", 2.0);
  const double map_resolution = declare_parameter<double>("map_resolution", 0.4);
  queue_size_ = declare_parameter<int>("queue_size", 10);
  max_processing_queue_size_ = static_cast<size_t>(declare_parameter<int>("max_processing_queue_size", 3));
  processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "lidar_cloud_filtered");
  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "odom");
  output_map_topic_ = declare_parameter<std::string>("output_map_topic", "map");

  // validate processing frequency to avoid division by zero in timer creation
  if (processing_frequency_ <= 0) {
    throw std::runtime_error("Invalid processing frequency: " + std::to_string(processing_frequency_));
  }

  lidar_mapping_.init(map_resolution);

  RCLCPP_INFO(get_logger(),
    "Parameters initialized - num_scan_lines: %d, scan_period: %.2f, max_dist: %.1f, min_dist: %.1f, "
    "map_resolution: %.2f, processing_frequency: %.1f Hz, max_processing_queue_size: %zu",
    lidar_param_.num_scan_lines, lidar_param_.scan_period, lidar_param_.max_distance,
    lidar_param_.min_distance, map_resolution, processing_frequency_, max_processing_queue_size_);
}

void LidarMapping::initialize_ros_components()
{
  // configure QoS profile for lidar point cloud transport
  rclcpp::QoS lidar_qos(queue_size_);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // create reentrant callback group for parallel execution
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // create message filters subscribers
  sub_lidar_cloud_.subscribe(this, input_cloud_topic_, lidar_qos.get_rmw_qos_profile());
  sub_odometry_.subscribe(this, input_odom_topic_, lidar_qos.get_rmw_qos_profile());

  // create exact time synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(queue_size_), sub_lidar_cloud_, sub_odometry_);
  sync_->registerCallback(
    std::bind(&LidarMapping::lidar_odom_callback, this, std::placeholders::_1, std::placeholders::_2));

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_map_topic_, lidar_qos);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&LidarMapping::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "ROS components initialized");
  RCLCPP_INFO(get_logger(), "Input cloud: %s, odom: %s, Output map: %s",
    input_cloud_topic_.c_str(), input_odom_topic_.c_str(), output_map_topic_.c_str());
}

void LidarMapping::lidar_odom_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_lock_);

    if (cloud_odom_buf_.size() >= max_processing_queue_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest cloud odom pair (queue size: %ld)",
        cloud_odom_buf_.size());
      cloud_odom_buf_.pop();
    }

    cloud_odom_buf_.push({cloud_msg, odom_msg});

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in lidar odom callback: %s", e.what());
  }
}

void LidarMapping::timer_callback()
{
  // skip if already processing — lidar_mapping_ is not thread-safe
  if (processing_in_progress_.load()) {
    return;
  }

  CloudOdomPair msg_pair;

  {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    if (cloud_odom_buf_.empty()) {
      return;
    }
    msg_pair = cloud_odom_buf_.front();
    cloud_odom_buf_.pop();
  }

  processing_in_progress_.store(true);

  rclcpp::Time pointcloud_time = msg_pair.first->header.stamp;

  try {
    process_mapping(msg_pair);

    if (pub_map_->get_subscription_count() > 0) {
      publish_mapping_result(pointcloud_time);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during lidar mapping: %s", e.what());
  }

  processing_in_progress_.store(false);
}

void LidarMapping::process_mapping(const CloudOdomPair& msg_pair)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg_pair.first, *pointcloud_in);

  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  current_pose.rotate(Eigen::Quaterniond(
    msg_pair.second->pose.pose.orientation.w,
    msg_pair.second->pose.pose.orientation.x,
    msg_pair.second->pose.pose.orientation.y,
    msg_pair.second->pose.pose.orientation.z));
  current_pose.pretranslate(Eigen::Vector3d(
    msg_pair.second->pose.pose.position.x,
    msg_pair.second->pose.pose.position.y,
    msg_pair.second->pose.pose.position.z));

  lidar_mapping_.update_current_points_to_map(pointcloud_in, current_pose);
}

void LidarMapping::publish_mapping_result(const rclcpp::Time& pointcloud_time)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = lidar_mapping_.get_map();
  sensor_msgs::msg::PointCloud2 points_msg;
  pcl::toROSMsg(*pc_map, points_msg);
  points_msg.header.stamp = pointcloud_time;
  points_msg.header.frame_id = "map";
  pub_map_->publish(points_msg);
}

} // namespace floam
