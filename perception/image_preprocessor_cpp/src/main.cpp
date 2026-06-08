// =============================================================================
// main.cpp — standalone entry point for image_preprocessor_cpp
// =============================================================================
// SingleThreadedExecutor: one subscription + one publication, no shared state,
// deterministic scheduling. Composable container is the normal deployment;
// this binary exists for standalone debugging and bag-replay use.
// =============================================================================

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "image_preprocessor_cpp/image_preprocessor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<image_preprocessor_cpp::ImagePreprocessorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
