// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <chrono>
#include <functional>

// local header
#include "floam/lidar_processing.hpp"


namespace floam
{

LidarProcessing::LidarProcessing()
  : Node("lidar_processing_node"), total_time_(0.0), total_frame_(0)
{
  // declare parameters
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 2.0;
  processing_frequency_ = 50.0;

  this->declare_parameter("scan_line", scan_line);
  this->declare_parameter("scan_period", scan_period);
  this->declare_parameter("vertical_angle", vertical_angle);
  this->declare_parameter("max_dist", max_dist);
  this->declare_parameter("min_dist", min_dist);
  this->declare_parameter("processing_frequency", processing_frequency_);
  this->declare_parameter("max_processing_queue_size", 3);

  // load from parameter if provided
  scan_line = this->get_parameter("scan_line").get_parameter_value().get<int>();
  scan_period = this->get_parameter("scan_period").get_parameter_value().get<double>();
  vertical_angle = this->get_parameter("vertical_angle").get_parameter_value().get<double>();
  max_dist = this->get_parameter("max_dist").get_parameter_value().get<double>();
  min_dist = this->get_parameter("min_dist").get_parameter_value().get<double>();
  processing_frequency_ = this->get_parameter("processing_frequency").get_parameter_value().get<double>();
  max_processing_queue_size_ = static_cast<size_t>(
    this->get_parameter("max_processing_queue_size").get_parameter_value().get<int>());

  lidar_param_.setScanPeriod(scan_period);
  lidar_param_.setVerticalAngle(vertical_angle);
  lidar_param_.setLines(scan_line);
  lidar_param_.setMaxDistance(max_dist);
  lidar_param_.setMinDistance(min_dist);

  lidar_processing_.init(lidar_param_);

  // create reentrant callback group for parallel execution
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "kitti/velo", 100,
    std::bind(&LidarProcessing::lidar_callback, this, std::placeholders::_1),
    sub_options);

  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_filtered", 100);
  edge_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_edge", 100);
  surf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surf", 100);

  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&LidarProcessing::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "Lidar processing node initialized successfully");
}

void LidarProcessing::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_lock_);

    if (point_cloud_buf_.size() >= max_processing_queue_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest point cloud (queue size: %ld)", point_cloud_buf_.size());
      point_cloud_buf_.pop();
    }

    point_cloud_buf_.push(lidar_cloud_msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in lidar callback: %s", e.what());
  }
}

void LidarProcessing::timer_callback()
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    if (point_cloud_buf_.empty()) {
      return;
    }
    msg = point_cloud_buf_.front();
    point_cloud_buf_.pop();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *pointcloud_in);
  rclcpp::Time pointcloud_time = msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

  try {
    process_lidar(pointcloud_in, pointcloud_edge, pointcloud_surf);

    if (!pointcloud_edge->empty() || !pointcloud_surf->empty()) {
      publish_lidar_result(pointcloud_edge, pointcloud_surf, pointcloud_time);
    } else {
      RCLCPP_WARN(get_logger(), "Lidar processing returned empty result");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during lidar processing: %s", e.what());
  }
}

void LidarProcessing::process_lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_surf)
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_edge,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_surf,
  const rclcpp::Time& pointcloud_time)
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

} // namespace floam
