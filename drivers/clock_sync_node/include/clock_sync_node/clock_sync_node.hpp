#ifndef CLOCK_SYNC_NODE__CLOCK_SYNC_NODE_HPP_
#define CLOCK_SYNC_NODE__CLOCK_SYNC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace clock_sync_node
{

class ClockSyncNode : public rclcpp::Node
{
public:
  explicit ClockSyncNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ClockSyncNode() override = default;

private:
  void publish_clock();

  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  builtin_interfaces::msg::Time msg_;

  static constexpr double PUBLISH_RATE_HZ = 10.0;
  static constexpr const char * TOPIC_NAME = "/clock_sync";
};

}  // namespace clock_sync_node

#endif  // CLOCK_SYNC_NODE__CLOCK_SYNC_NODE_HPP_
