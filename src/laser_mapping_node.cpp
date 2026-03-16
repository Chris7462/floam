// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/laser_mapping.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lmn_ptr {std::make_shared<floam::LaserMapping>()};
  std::thread laser_mapping_process(&floam::LaserMapping::laser_mapping, lmn_ptr);
  rclcpp::spin(lmn_ptr);
  rclcpp::shutdown();

  return 0;
}
