#ifndef SAFETY__WATCHDOG_NODE_HPP_
#define SAFETY__WATCHDOG_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "bfmc_msgs/msg/trajectory.hpp" // STRICT CONTRACT COMPLIANCE

namespace safety
{

class WatchdogNode : public rclcpp::Node
{
public:
  explicit WatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<bfmc_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_armed_{false};
  rclcpp::Time last_msg_time_;

  void trajectory_callback(const bfmc_msgs::msg::Trajectory::SharedPtr msg);
  void timer_callback();
};

} // namespace safety

#endif // SAFETY__WATCHDOG_NODE_HPP_
