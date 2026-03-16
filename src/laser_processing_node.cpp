// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/laser_processing.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lpn_ptr {std::make_shared<floam::LaserProcessing>()};
  std::thread laser_processing_process(&floam::LaserProcessing::laser_processing, lpn_ptr);
  rclcpp::spin(lpn_ptr);
  rclcpp::shutdown();

  return 0;
}
