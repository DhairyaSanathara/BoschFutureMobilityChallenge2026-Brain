#include "safety/watchdog_node.hpp"
#include <chrono>
#include <functional>
#include <memory>

namespace safety
{

WatchdogNode::WatchdogNode(const rclcpp::NodeOptions & options)
: Node("watchdog_node", options), is_armed_(false)
{
  pub_estop_ = this->create_publisher<std_msgs::msg::Bool>(
      "/safety/emergency_stop",
      rclcpp::QoS(10).reliable());

  // Listen to the strict ROS trajectory message output as our heartbeat
  sub_traj_ = this->create_subscription<bfmc_msgs::msg::Trajectory>(
      "/planning/trajectory",
      rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&WatchdogNode::trajectory_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WatchdogNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "[Watchdog] Initialized. Waiting for planner heartbeat...");
}

void WatchdogNode::trajectory_callback(const bfmc_msgs::msg::Trajectory::SharedPtr msg)
{
  (void)msg; // We only care that the message arrived, payload doesn't matter here
  if (!is_armed_) {
    RCLCPP_INFO(this->get_logger(), "[Watchdog] First trajectory received. ARMED.");
    is_armed_ = true;
  }
  last_msg_time_ = this->get_clock()->now();
}

void WatchdogNode::timer_callback()
{
  // Do not trigger E-Stop if the system is simply still booting
  if (!is_armed_) return;

  auto now = this->get_clock()->now();
  double elapsed = (now - last_msg_time_).seconds();

  std_msgs::msg::Bool msg;
  if (elapsed > 0.5) { // 500ms timeout
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "[Watchdog] TRAJECTORY TIMEOUT! Elapsed: %.2fs. Triggering E-STOP!", elapsed);
    msg.data = true;
  } else {
    msg.data = false;
  }
  pub_estop_->publish(msg);
}

} // namespace safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety::WatchdogNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
