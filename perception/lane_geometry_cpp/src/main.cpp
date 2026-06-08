// =============================================================================
// main.cpp -- standalone entry point for lane_geometry_cpp
// =============================================================================
// SingleThreadedExecutor: this node has one subscription and one publication
// (plus lifecycle services). MultiThreaded scheduling adds mutex overhead
// without any concurrency benefit.
// =============================================================================

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lane_geometry_cpp/lane_geometry_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<lane_geometry_cpp::LaneGeometryNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
