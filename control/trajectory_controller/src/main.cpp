// main.cpp
// ──────────────────────────────────────────────────────────────
// NMPC Trajectory Controller – BFMC 2026
// Team OdyssiX / ORIS
//
// SingleThreadedExecutor as mandated by architecture freeze.
// Lifecycle transitions handled EXTERNALLY by nav2_lifecycle_manager.
// ──────────────────────────────────────────────────────────────

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "trajectory_controller/trajectory_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<bfmc_control::TrajectoryControllerNode>();

  // DO NOT manually trigger transitions here.
  // The node will boot in the UNCONFIGURED state and wait for the Lifecycle Manager.

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
