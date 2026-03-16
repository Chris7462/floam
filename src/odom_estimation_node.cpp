// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/odom_estimation.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto oen_ptr {std::make_shared<floam::OdomEstimation>()};
  std::thread odom_estimation_process(&floam::OdomEstimation::odom_estimation, oen_ptr);
  rclcpp::spin(oen_ptr);
  rclcpp::shutdown();

  return 0;
}
