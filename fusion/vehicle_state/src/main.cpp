#include "rclcpp/rclcpp.hpp"
#include "vehicle_state/vehicle_state_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<bfmc_fusion::VehicleStateNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
