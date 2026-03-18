#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
#include "floam_core/lidar_mapping.hpp"


namespace floam
{

class LidarMapping : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LidarMapping node
   */
  LidarMapping();

  /**
   * @brief Destructor for LidarMapping node
   */
  ~LidarMapping();

private:
  // type aliases for complex types
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2,
    nav_msgs::msg::Odometry>;
  using CloudOdomPair = std::pair<
    sensor_msgs::msg::PointCloud2::ConstSharedPtr,
    nav_msgs::msg::Odometry::ConstSharedPtr>;

  /**
   * @brief Initialize node parameters
   */
  void initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, synchronizer and timers
   */
  void initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming point cloud and odometry
   * @param cloud_msg Incoming filtered point cloud message
   * @param odom_msg Incoming odometry message
   */
  void lidar_odom_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  /**
   * @brief Timer callback for processing at regular intervals
   */
  void timer_callback();

  /**
   * @brief Process lidar mapping from buffered point cloud and odometry pair
   * @param msg_pair Synchronized pair of point cloud and odometry messages
   */
  void process_mapping(const CloudOdomPair& msg_pair);

  /**
   * @brief Publish mapping results
   * @param pointcloud_time Timestamp of the point cloud
   */
  void publish_mapping_result(const rclcpp::Time& pointcloud_time);

private:
  // ROS2 components
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_cloud_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odometry_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Core processing
  floam_core::LidarMapping lidar_mapping_;
  floam_core::Lidar lidar_param_;

  // Buffer — synchronized pairs of point cloud and odometry
  std::queue<CloudOdomPair> cloud_odom_buf_;
  std::mutex mutex_lock_;

  // Parameters
  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_map_topic_;
  int queue_size_;
  size_t max_processing_queue_size_;
  double processing_frequency_;

  // State
  std::atomic<bool> processing_in_progress_;
};

} // namespace floam
