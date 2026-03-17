// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <thread>
#include <chrono>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/lidar_mapping.hpp"


namespace floam
{

LidarMapping::LidarMapping()
  : Node("lidar_mapping_node")
{
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 2.0;
  double map_resolution = 0.4;

  this->declare_parameter("scan_line", scan_line);
  this->declare_parameter("scan_period", scan_period);
  this->declare_parameter("vertical_angle", vertical_angle);
  this->declare_parameter("max_dist", max_dist);
  this->declare_parameter("min_dist", min_dist);
  this->declare_parameter("map_resolution", map_resolution);

  // load from parameter if provided
  scan_line = this->get_parameter("scan_line").get_parameter_value().get<int>();
  scan_period = this->get_parameter("scan_period").get_parameter_value().get<double>();
  vertical_angle = this->get_parameter("vertical_angle").get_parameter_value().get<double>();
  max_dist = this->get_parameter("max_dist").get_parameter_value().get<double>();
  min_dist = this->get_parameter("min_dist").get_parameter_value().get<double>();
  map_resolution = this->get_parameter("map_resolution").get_parameter_value().get<double>();

  lidar_param_.setScanPeriod(scan_period);
  lidar_param_.setVerticalAngle(vertical_angle);
  lidar_param_.setLines(scan_line);
  lidar_param_.setMaxDistance(max_dist);
  lidar_param_.setMinDistance(min_dist);

  lidar_mapping_.init(map_resolution);

  sub_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points_filtered", 100, std::bind(&LidarMapping::lidar_handler, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 100, std::bind(&LidarMapping::odom_callback, this, std::placeholders::_1));

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);
}

void LidarMapping::lidar_mapping()
{
  while (1) {
    if (!odometry_buf_.empty() && !point_cloud_buf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!point_cloud_buf_.empty() && point_cloud_buf_.front()->header.stamp.sec < odometry_buf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period) {
        point_cloud_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned error and pointcloud discarded, pls check your data --> lidar mapping node");
        mutex_lock_.unlock();
        continue;
      }

      if (!odometry_buf_.empty() && odometry_buf_.front()->header.stamp.sec < point_cloud_buf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period) {
        odometry_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with path final, pls check your data --> lidar mapping node");
        mutex_lock_.unlock();
        continue;
      }

      // if time aligned
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*point_cloud_buf_.front(), *pointcloud_in);
      rclcpp::Time pointcloud_time = (point_cloud_buf_.front())->header.stamp;

      Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
      current_pose.rotate(Eigen::Quaterniond(odometry_buf_.front()->pose.pose.orientation.w, odometry_buf_.front()->pose.pose.orientation.x, odometry_buf_.front()->pose.pose.orientation.y, odometry_buf_.front()->pose.pose.orientation.z));
      current_pose.pretranslate(Eigen::Vector3d(odometry_buf_.front()->pose.pose.position.x, odometry_buf_.front()->pose.pose.position.y, odometry_buf_.front()->pose.pose.position.z));
      point_cloud_buf_.pop();
      odometry_buf_.pop();
      mutex_lock_.unlock();

      lidar_mapping_.updateCurrentPointsToMap(pointcloud_in, current_pose);

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = lidar_mapping_.getMap();
      sensor_msgs::msg::PointCloud2 points_msg;
      pcl::toROSMsg(*pc_map, points_msg);
      points_msg.header.stamp = pointcloud_time;
      points_msg.header.frame_id = "map";
      pub_map_->publish(points_msg);
    }

    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void LidarMapping::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  mutex_lock_.lock();
  odometry_buf_.push(odom_msg);
  mutex_lock_.unlock();
}

void LidarMapping::lidar_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg)
{
  mutex_lock_.lock();
  point_cloud_buf_.push(lidar_cloud_msg);
  mutex_lock_.unlock();
}

} // namespace floam
