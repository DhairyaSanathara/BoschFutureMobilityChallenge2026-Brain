#include "local_path_planner_cpp/local_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_path_planner_cpp::LocalPathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
