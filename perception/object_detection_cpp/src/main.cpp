// =============================================================================
// main.cpp — object_detection_node entry point.
// MultiThreadedExecutor with 2 threads so depth + camera_info callbacks can
// run while the RGB callback is mid-inference.
// =============================================================================

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "object_detection_cpp/object_detection_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<object_detection_cpp::ObjectDetectionNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
