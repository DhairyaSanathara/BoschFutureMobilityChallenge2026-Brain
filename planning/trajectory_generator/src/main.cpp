// Trajectory Generator – BFMC 2026
// Team OdyssiX

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "trajectory_generator/trajectory_generator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<trajectory_generator::TrajectoryGeneratorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
