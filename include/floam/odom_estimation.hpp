#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

// message filters header
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// c++ header
#include <atomic>
#include <mutex>
#include <queue>

// local header
#include "floam_core/lidar.hpp"
#include "floam_core/odom_estimation.hpp"


namespace floam
{

class OdomEstimation : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for OdomEstimation node
   */
  OdomEstimation();

  /**
   * @brief Destructor for OdomEstimation node
   */
  ~OdomEstimation();

private:
  // type aliases for complex types
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;
  using PointCloudPair = std::pair<
    sensor_msgs::msg::PointCloud2::ConstSharedPtr,
    sensor_msgs::msg::PointCloud2::ConstSharedPtr>;

  /**
   * @brief Initialize node parameters
   */
  void initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, synchronizer and timers
   */
  void initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming edge and surf point clouds
   * @param edge_msg Incoming edge point cloud message
   * @param surf_msg Incoming surf point cloud message
   */
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr surf_msg);

  /**
   * @brief Timer callback for processing point clouds at regular intervals
   */
  void timer_callback();

  /**
   * @brief Process odometry estimation from a synchronized point cloud pair
   * @param msg_pair Synchronized pair of edge and surf point cloud messages
   */
  void process_odom(const PointCloudPair & msg_pair);

  /**
   * @brief Publish odometry results
   * @param pointcloud_time Timestamp of the point cloud
   */
  void publish_odom_result(const rclcpp::Time& pointcloud_time);

private:
  // ROS2 components
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_edge_lidar_cloud_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_surf_lidar_cloud_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_laser_path_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

  // Callback group for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Core processing
  floam_core::OdomEstimation odom_estimation_;
  floam_core::Lidar lidar_param_;

  // Point cloud buffer — synchronized pairs of edge and surf
  std::queue<PointCloudPair> point_cloud_buf_;
  std::mutex mutex_lock_;

  // Path
  nav_msgs::msg::Path laser_path_;

  // Parameters
  std::string input_edge_topic_;
  std::string input_surf_topic_;
  std::string output_odom_topic_;
  std::string output_path_topic_;
  int queue_size_;
  size_t max_processing_queue_size_;
  double processing_frequency_;

  // State
  std::atomic<bool> processing_in_progress_;
  bool is_odom_inited_;
  double total_time_;
  int total_frame_;
};

} // namespace floam
