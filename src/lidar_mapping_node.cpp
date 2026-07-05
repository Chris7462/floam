// ros header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// c++ header
#include <stdexcept>

// local header
#include "floam/lidar_mapping.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<floam::LidarMapping>();

    // EventsCBGExecutor: uses 10-15% less CPU than MultiThreadedExecutor,
    // supports multiple ROS time sources, and manages threading internally.
    rclcpp::executors::EventsCBGExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_mapping_node"),
      "Failed to initialize: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
