#include "clock_sync_node/clock_sync_node.hpp"

namespace clock_sync_node
{

ClockSyncNode::ClockSyncNode(const rclcpp::NodeOptions & options)
: Node("clock_sync_node", options)
{
  // RELIABLE QoS for system timing
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
               .reliability(rclcpp::ReliabilityPolicy::Reliable)
               .durability(rclcpp::DurabilityPolicy::Volatile);

  publisher_ =
    this->create_publisher<builtin_interfaces::msg::Time>(
      TOPIC_NAME, qos);

  msg_.sec = 0;
  msg_.nanosec = 0;

  auto period =
    std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ);

  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ClockSyncNode::publish_clock, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Clock Sync Node started | topic=%s | rate=%.0f Hz",
    TOPIC_NAME,
    PUBLISH_RATE_HZ);
}

void ClockSyncNode::publish_clock()
{
  auto now = this->get_clock()->now();

  msg_.sec = now.seconds();
  msg_.nanosec =
    now.nanoseconds() % 1000000000ULL;

  publisher_->publish(msg_);
}

}  // namespace clock_sync_node
