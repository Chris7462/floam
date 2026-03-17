#pragma once

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// c++ lib
#include <mutex>
#include <queue>

// local lib
#include "floam_core/lidar.hpp"
#include "floam_core/lidar_processing.hpp"


namespace floam
{

class LidarProcessing : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LidarProcessing node
   */
  LidarProcessing();

  /**
   * @brief Destructor for LidarProcessing node
   */
  ~LidarProcessing();

private:
  /**
   * @brief Initialize node parameters
   * @throws std::runtime_error if parameter validation fails
   */
  void initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, timers and callback groups
   */
  void initialize_ros_components();

  /**
   * @brief Callback function for incoming lidar point clouds
   * @param lidar_cloud_msg Incoming point cloud message
   */
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg);

  /**
   * @brief Timer callback for processing point clouds at regular intervals
   */
  void timer_callback();

  /**
   * @brief Extract edge and surface features from input point cloud
   * @param pointcloud_in Input point cloud
   * @param pointcloud_edge Output edge feature point cloud
   * @param pointcloud_surf Output surface feature point cloud
   */
  void process_lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf);

  /**
   * @brief Publish lidar processing results
   * @param pointcloud_edge Edge feature point cloud
   * @param pointcloud_surf Surface feature point cloud
   * @param pointcloud_time Timestamp of the original point cloud
   */
  void publish_lidar_result(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf,
    const rclcpp::Time & pointcloud_time);

private:
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Core processing
  floam_core::LidarProcessing lidar_processing_;
  floam_core::Lidar lidar_param_;

  // Point cloud buffer
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_buf_;
  std::mutex mutex_lock_;

  // Parameters
  double processing_frequency_;
  size_t max_processing_queue_size_;

  // Timing
  double total_time_;
  int total_frame_;
};

} // namespace floam
