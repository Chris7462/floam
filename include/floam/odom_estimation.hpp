#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

// c++ header
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

  /**
   * @brief Main odometry estimation loop
   */
  void odom_estimation();

private:
  /**
   * @brief Initialize node parameters
   * @throws std::runtime_error if parameter validation fails
   */
  void initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers and callback groups
   */
  void initialize_ros_components();

  /**
   * @brief Callback for incoming edge point cloud
   * @param lidar_cloud_msg Incoming edge point cloud message
   */
  void lidar_edge_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg);

  /**
   * @brief Callback for incoming surf point cloud
   * @param lidar_cloud_msg Incoming surf point cloud message
   */
  void lidar_surf_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg);

private:
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_edge_lidar_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_surf_lidar_cloud_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_laser_path_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

  // Core processing
  floam_core::OdomEstimation odom_estimation_;
  floam_core::Lidar lidar_param_;

  // Point cloud buffers
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_edge_buf_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_surf_buf_;
  std::mutex mutex_lock_;

  // Path
  nav_msgs::msg::Path laser_path_;

  // State
  bool is_odom_inited_;
  double total_time_;
  int total_frame_;
};

} // namespace floam
