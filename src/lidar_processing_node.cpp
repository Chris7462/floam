// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <stdexcept>

// local header
#include "floam/lidar_processing.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<floam::LidarProcessing>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_processing_node"),
      "Failed to initialize: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
