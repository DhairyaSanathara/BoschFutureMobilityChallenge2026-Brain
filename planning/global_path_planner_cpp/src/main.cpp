#include "global_path_planner_cpp/planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_path_planner_cpp::GlobalPathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
