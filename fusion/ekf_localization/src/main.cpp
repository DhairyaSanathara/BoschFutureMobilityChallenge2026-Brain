#include "rclcpp/rclcpp.hpp"
#include "ekf_localization/ekf_localization_node.hpp"

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<bfmc_fusion::EkfLocalizationNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
