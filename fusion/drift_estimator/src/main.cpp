#include "rclcpp/rclcpp.hpp"
#include "drift_estimator/drift_estimator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<bfmc_fusion::DriftEstimatorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
