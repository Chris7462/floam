#pragma once

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// c++ lib
#include <mutex>
#include <queue>

// local lib
#include "floam_core/lidar.hpp"
#include "floam_core/laser_processing.hpp"


namespace floam
{

class LaserProcessing : public rclcpp::Node
{
  public:
    LaserProcessing();
    void laser_processing();

  private:
    floam_core::LaserProcessing laserProcessing_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudBuf_;
    std::mutex mutex_lock_;
    floam_core::Lidar lidar_param_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered_;

    void velodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg);

    double total_time_;
    int total_frame_;
};

} // namespace floam
