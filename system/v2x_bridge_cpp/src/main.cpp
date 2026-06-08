#include "v2x_bridge_cpp/v2x_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<v2x_bridge_cpp::V2xBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
