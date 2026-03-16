// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/lidar_mapping.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lmn_ptr {std::make_shared<floam::LidarMapping>()};
  std::thread lidar_mapping_process(&floam::LidarMapping::lidar_mapping, lmn_ptr);
  rclcpp::spin(lmn_ptr);
  rclcpp::shutdown();

  return 0;
}
