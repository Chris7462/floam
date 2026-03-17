// ros header
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// c++ header
#include <thread>
#include <chrono>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/odom_estimation.hpp"


namespace floam
{

OdomEstimation::OdomEstimation()
  : Node("odom_estimation_node"), laser_path_{}, is_odom_inited_(false), total_time_(0.0), total_frame_(0)
{
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 3.0;
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

  odom_estimation_.init(map_resolution);

  sub_edge_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_edge", 100, std::bind(&OdomEstimation::lidar_edge_handler, this, std::placeholders::_1));
  sub_surf_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_surf", 100, std::bind(&OdomEstimation::lidar_surf_handler, this, std::placeholders::_1));

  pub_laser_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  pub_laser_path_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 100);

  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OdomEstimation::odom_estimation()
{
  while (1) {
    if (!point_cloud_edge_buf_.empty() && !point_cloud_surf_buf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!point_cloud_surf_buf_.empty() && (point_cloud_surf_buf_.front()->header.stamp.sec < point_cloud_edge_buf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period)) {
        point_cloud_surf_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock_.unlock();
        continue;
      }

      if (!point_cloud_edge_buf_.empty() && (point_cloud_edge_buf_.front()->header.stamp.sec < point_cloud_surf_buf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period)) {
        point_cloud_edge_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock_.unlock();
        continue;
      }

      // if time aligned
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*point_cloud_edge_buf_.front(), *pointcloud_edge_in);
      pcl::fromROSMsg(*point_cloud_surf_buf_.front(), *pointcloud_surf_in);

      rclcpp::Time pointcloud_time = point_cloud_surf_buf_.front()->header.stamp;
      point_cloud_edge_buf_.pop();
      point_cloud_surf_buf_.pop();
      mutex_lock_.unlock();

      if (is_odom_inited_ == false) {
        odom_estimation_.init_map_with_points(pointcloud_edge_in, pointcloud_surf_in);
        is_odom_inited_ = true;
        RCLCPP_INFO(this->get_logger(), "odom inited");
      } else {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        odom_estimation_.update_points_to_map(pointcloud_edge_in, pointcloud_surf_in);
        end = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame_++;
        float time_temp = elapsed_seconds.count() * 1000;
        total_time_ += time_temp;
        RCLCPP_INFO(this->get_logger(), "average odom estimation time %f ms\n", total_time_/total_frame_);
      }

      Eigen::Quaterniond q_current(odom_estimation_.odom.rotation());
      Eigen::Vector3d t_current = odom_estimation_.odom.translation();

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = pointcloud_time;
      t.header.frame_id = "map";
      t.child_frame_id = "base_link";

      t.transform.translation.x = t_current.x();
      t.transform.translation.y = t_current.y();
      t.transform.translation.z = t_current.z();

      tf2::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      br_->sendTransform(t);

      // publish odometry
      nav_msgs::msg::Odometry laser_odometry;
      laser_odometry.header.frame_id = "map";
      laser_odometry.child_frame_id = "base_link";
      laser_odometry.header.stamp = pointcloud_time;
      laser_odometry.pose.pose.orientation.x = q_current.x();
      laser_odometry.pose.pose.orientation.y = q_current.y();
      laser_odometry.pose.pose.orientation.z = q_current.z();
      laser_odometry.pose.pose.orientation.w = q_current.w();
      laser_odometry.pose.pose.position.x = t_current.x();
      laser_odometry.pose.pose.position.y = t_current.y();
      laser_odometry.pose.pose.position.z = t_current.z();
      pub_laser_odometry_->publish(laser_odometry);

      // publish path
      geometry_msgs::msg::PoseStamped laser_pose;
      laser_pose.header = laser_odometry.header;
      laser_pose.pose = laser_odometry.pose.pose;

      laser_path_.header = laser_odometry.header;
      laser_path_.poses.push_back(laser_pose);
      pub_laser_path_->publish(laser_path_);
    }

    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void OdomEstimation::lidar_surf_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg)
{
  mutex_lock_.lock();
  point_cloud_surf_buf_.push(lidar_cloud_msg);
  mutex_lock_.unlock();
}

void OdomEstimation::lidar_edge_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_cloud_msg)
{
  mutex_lock_.lock();
  point_cloud_edge_buf_.push(lidar_cloud_msg);
  mutex_lock_.unlock();
}

} // namespace floam
