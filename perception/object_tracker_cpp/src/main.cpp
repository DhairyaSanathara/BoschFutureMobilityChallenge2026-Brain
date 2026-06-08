#include "object_tracker_cpp/object_tracker_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<object_tracker_cpp::ObjectTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
