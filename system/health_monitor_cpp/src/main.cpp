#include "health_monitor_cpp/health_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bfmc_system::HealthMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
