#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

// c++ header
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
  LidarMapping();
  void lidar_mapping();

private:
  floam_core::LidarMapping lidar_mapping_;
  std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buf_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_buf_;
  std::mutex mutex_lock_;
  floam_core::Lidar lidar_param_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void lidar_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg);
};

} // namespace floam
