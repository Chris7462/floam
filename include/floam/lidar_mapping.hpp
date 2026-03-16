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
  floam_core::LidarMapping lidarMapping_;
  std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudBuf_;
  std::mutex mutex_lock_;
  floam_core::Lidar lidar_param_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidarCloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap_;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg);
  void lidarHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidarCloudMsg);
};

} // namespace floam
