// =============================================================================
// main.cpp -- Free Space Node entry point (BFMC PR3)
// =============================================================================
// FINAL PRODUCTION FREEZE v3
//
// [H-2] SingleThreadedExecutor: deterministic scheduling, no thread contention,
//        no scheduling jitter. Matches single-subscriber single-pipeline design.
// =============================================================================
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "free_space_cpp/free_space_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<free_space_cpp::FreeSpaceNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
