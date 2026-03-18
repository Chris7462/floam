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
  const int scan_line = declare_parameter<int>("scan_line", 64);
  const double scan_period = declare_parameter<double>("scan_period", 0.1);
  const double vertical_angle = declare_parameter<double>("vertical_angle", 2.0);
  const double max_dist = declare_parameter<double>("max_dist", 90.0);
  const double min_dist = declare_parameter<double>("min_dist", 2.0);
  const double map_resolution = declare_parameter<double>("map_resolution", 0.4);

  // set lidar parameters
  lidar_param_.scan_period = scan_period;
  lidar_param_.vertical_angle = vertical_angle;
  lidar_param_.num_scan_lines = scan_line;
  lidar_param_.max_distance = max_dist;
  lidar_param_.min_distance = min_dist;

  lidar_mapping_.init(map_resolution);

  // configure QoS profile for lidar point cloud transport
  rclcpp::QoS lidar_qos(10);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  sub_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points_filtered", lidar_qos,
    std::bind(&LidarMapping::lidar_handler, this, std::placeholders::_1));

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", lidar_qos,
    std::bind(&LidarMapping::odom_callback, this, std::placeholders::_1));

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", lidar_qos);
}

void LidarMapping::lidar_mapping()
{
  while (1) {
    if (!odometry_buf_.empty() && !point_cloud_buf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!point_cloud_buf_.empty() &&
        (rclcpp::Time(point_cloud_buf_.front()->header.stamp) < rclcpp::Time(odometry_buf_.front()->header.stamp) - rclcpp::Duration::from_seconds(0.5 * lidar_param_.scan_period))) {
        point_cloud_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned error and pointcloud discarded, pls check your data --> lidar mapping node");
        mutex_lock_.unlock();
        continue;
      }

      if (!odometry_buf_.empty() &&
        (rclcpp::Time(odometry_buf_.front()->header.stamp) < rclcpp::Time(point_cloud_buf_.front()->header.stamp) - rclcpp::Duration::from_seconds(0.5 * lidar_param_.scan_period))) {
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

      lidar_mapping_.update_current_points_to_map(pointcloud_in, current_pose);

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = lidar_mapping_.get_map();
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
