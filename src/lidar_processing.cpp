// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <chrono>
#include <functional>
#include <stdexcept>

// local header
#include "floam/lidar_processing.hpp"


namespace floam
{

LidarProcessing::LidarProcessing()
: Node("lidar_processing_node"), processing_in_progress_(false),
  total_time_(0.0), total_frame_(0)
{
  initialize_parameters();
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "Lidar processing node initialized successfully");
}

LidarProcessing::~LidarProcessing()
{
  RCLCPP_INFO(get_logger(), "Lidar processing node shutting down");
}

void LidarProcessing::initialize_parameters()
{
  // declare and load parameters
  lidar_param_.scan_period = declare_parameter<double>("scan_period", 0.1);
  lidar_param_.vertical_angle = declare_parameter<double>("vertical_angle", 2.0);
  lidar_param_.num_scan_lines = declare_parameter<int>("num_scan_lines", 64);
  lidar_param_.max_distance = declare_parameter<double>("max_dist", 90.0);
  lidar_param_.min_distance = declare_parameter<double>("min_dist", 2.0);
  processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
  max_processing_queue_size_ =
    static_cast<size_t>(declare_parameter<int>("max_processing_queue_size", 3));
  queue_size_ = declare_parameter<int>("queue_size", 10);
  input_topic_ = declare_parameter<std::string>("input_topic", "kitti/velo");
  output_filtered_topic_ = declare_parameter<std::string>("output_filtered_topic",
      "lidar_cloud_filtered");
  output_edge_topic_ = declare_parameter<std::string>("output_edge_topic", "lidar_cloud_edge");
  output_surf_topic_ = declare_parameter<std::string>("output_surf_topic", "lidar_cloud_surf");

  // validate processing frequency to avoid division by zero in timer creation
  if (processing_frequency_ <= 0) {
    throw std::runtime_error("Invalid processing frequency: " +
        std::to_string(processing_frequency_));
  }

  lidar_processing_.init(lidar_param_);

  RCLCPP_INFO(get_logger(),
    "Parameters initialized - num_scan_lines: %d, scan_period: %.2f, max_dist: %.1f, min_dist: %.1f, "
    "processing_frequency: %.1f Hz, max_processing_queue_size: %zu",
    lidar_param_.num_scan_lines, lidar_param_.scan_period, lidar_param_.max_distance,
    lidar_param_.min_distance, processing_frequency_, max_processing_queue_size_);
}

void LidarProcessing::initialize_ros_components()
{
  // configure QoS profile for lidar point cloud transport
  rclcpp::QoS lidar_qos(queue_size_);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // create reentrant callback group for parallel execution
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, lidar_qos,
    std::bind(&LidarProcessing::lidar_callback, this, std::placeholders::_1),
    sub_options);

  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_filtered_topic_, lidar_qos);
  edge_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_edge_topic_, lidar_qos);
  surf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_surf_topic_, lidar_qos);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&LidarProcessing::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "ROS components initialized");
  RCLCPP_INFO(get_logger(), "Input: %s, Output filtered: %s, edge: %s, surf: %s",
    input_topic_.c_str(), output_filtered_topic_.c_str(),
    output_edge_topic_.c_str(), output_surf_topic_.c_str());
}

void LidarProcessing::lidar_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_lock_);

    if (point_cloud_buf_.size() >= max_processing_queue_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest point cloud (queue size: %ld)",
          point_cloud_buf_.size());
      point_cloud_buf_.pop();
    }

    point_cloud_buf_.push(lidar_cloud_msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in lidar callback: %s", e.what());
  }
}

void LidarProcessing::timer_callback()
{
  // skip if already processing — lidar_processing_ is not thread-safe
  if (processing_in_progress_.load()) {
    return;
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    if (point_cloud_buf_.empty()) {
      return;
    }
    msg = point_cloud_buf_.front();
    point_cloud_buf_.pop();
  }

  processing_in_progress_.store(true);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *pointcloud_in);
  rclcpp::Time pointcloud_time = msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

  try {
    process_lidar(pointcloud_in, pointcloud_edge, pointcloud_surf);

    if (!pointcloud_edge->empty() || !pointcloud_surf->empty()) {
      if (edge_pub_->get_subscription_count() > 0 ||
        surf_pub_->get_subscription_count() > 0 ||
        filtered_pub_->get_subscription_count() > 0)
      {
        publish_lidar_result(pointcloud_edge, pointcloud_surf, pointcloud_time);
      }
    } else {
      RCLCPP_WARN(get_logger(), "Lidar processing returned empty result");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during lidar processing: %s", e.what());
  }

  processing_in_progress_.store(false);
}

void LidarProcessing::process_lidar(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf)
{
  auto start = std::chrono::system_clock::now();
  lidar_processing_.feature_extraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end - start;
  total_frame_++;
  total_time_ += elapsed_seconds.count() * 1000;
  //RCLCPP_INFO(get_logger(), "Average lidar processing time %f ms", total_time_/total_frame_);
}

void LidarProcessing::publish_lidar_result(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf,
  const rclcpp::Time & pointcloud_time)
{
  sensor_msgs::msg::PointCloud2 lidar_cloud_filtered_msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  *pointcloud_filtered += *pointcloud_edge;
  *pointcloud_filtered += *pointcloud_surf;
  pcl::toROSMsg(*pointcloud_filtered, lidar_cloud_filtered_msg);
  lidar_cloud_filtered_msg.header.stamp = pointcloud_time;
  lidar_cloud_filtered_msg.header.frame_id = "base_link";
  filtered_pub_->publish(lidar_cloud_filtered_msg);

  sensor_msgs::msg::PointCloud2 edge_points_msg;
  pcl::toROSMsg(*pointcloud_edge, edge_points_msg);
  edge_points_msg.header.stamp = pointcloud_time;
  edge_points_msg.header.frame_id = "base_link";
  edge_pub_->publish(edge_points_msg);

  sensor_msgs::msg::PointCloud2 surf_points_msg;
  pcl::toROSMsg(*pointcloud_surf, surf_points_msg);
  surf_points_msg.header.stamp = pointcloud_time;
  surf_points_msg.header.frame_id = "base_link";
  surf_pub_->publish(surf_points_msg);
}

}  // namespace floam
