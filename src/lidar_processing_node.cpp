// ros header
#include <rclcpp/rclcpp.hpp>

// local header
#include "floam/lidar_processing.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<floam::LidarProcessing>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
