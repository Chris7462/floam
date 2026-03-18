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
  initialize_parameters();
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "Odom estimation node initialized successfully");
}

OdomEstimation::~OdomEstimation()
{
  RCLCPP_INFO(get_logger(), "Odom estimation node shutting down");
}

void OdomEstimation::initialize_parameters()
{
  // declare and load parameters directly into lidar_param_
  lidar_param_.scan_period = declare_parameter<double>("scan_period", 0.1);
  lidar_param_.vertical_angle = declare_parameter<double>("vertical_angle", 2.0);
  lidar_param_.num_scan_lines = declare_parameter<int>("num_scan_lines", 64);
  lidar_param_.max_distance = declare_parameter<double>("max_dist", 90.0);
  lidar_param_.min_distance = declare_parameter<double>("min_dist", 2.0);
  const double map_resolution = declare_parameter<double>("map_resolution", 0.4);

  odom_estimation_.init(map_resolution);

  RCLCPP_INFO(get_logger(),
    "Parameters initialized - num_scan_lines: %d, scan_period: %.2f, max_dist: %.1f, min_dist: %.1f, "
    "map_resolution: %.2f",
    lidar_param_.num_scan_lines, lidar_param_.scan_period, lidar_param_.max_distance,
    lidar_param_.min_distance, map_resolution);
}

void OdomEstimation::initialize_ros_components()
{
  // configure QoS profile for lidar point cloud transport
  rclcpp::QoS lidar_qos(10);
  lidar_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  lidar_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  lidar_qos.history(rclcpp::HistoryPolicy::KeepLast);

  sub_edge_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_edge", lidar_qos,
    std::bind(&OdomEstimation::lidar_edge_handler, this, std::placeholders::_1));

  sub_surf_lidar_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_surf", lidar_qos,
    std::bind(&OdomEstimation::lidar_surf_handler, this, std::placeholders::_1));

  pub_laser_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", lidar_qos);
  pub_laser_path_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", lidar_qos);

  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "ROS components initialized");
}

void OdomEstimation::odom_estimation()
{
  while (1) {
    if (!point_cloud_edge_buf_.empty() && !point_cloud_surf_buf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!point_cloud_surf_buf_.empty() &&
        (rclcpp::Time(point_cloud_surf_buf_.front()->header.stamp) < rclcpp::Time(point_cloud_edge_buf_.front()->header.stamp) - rclcpp::Duration::from_seconds(0.5 * lidar_param_.scan_period))) {
        point_cloud_surf_buf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock_.unlock();
        continue;
      }

      if (!point_cloud_edge_buf_.empty() &&
        (rclcpp::Time(point_cloud_edge_buf_.front()->header.stamp) < rclcpp::Time(point_cloud_surf_buf_.front()->header.stamp) - rclcpp::Duration::from_seconds(0.5 * lidar_param_.scan_period))) {
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
