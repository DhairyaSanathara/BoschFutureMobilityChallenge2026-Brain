#ifndef VEHICLE_STATE__VEHICLE_STATE_NODE_HPP_
#define VEHICLE_STATE__VEHICLE_STATE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"

namespace bfmc_fusion
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VehicleStateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit VehicleStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /* Lifecycle */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  /* Internal */
  // Callback-driven publish (no timer): every /vehicle/state_raw sample is
  // EMA-filtered and re-published as /vehicle/state in the same call, so the
  // node's own latency contribution is just the cost of one filter step.
  void state_raw_callback(const bfmc_msgs::msg::VehicleState::SharedPtr msg);

  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr raw_sub_;
  rclcpp_lifecycle::LifecyclePublisher<bfmc_msgs::msg::VehicleState>::SharedPtr state_pub_;

  bfmc_msgs::msg::VehicleState filtered_msg_;

  // Lifecycle-active gate. Subscriptions exist from on_configure onward, but
  // we must not publish until on_activate, and we must drop work after
  // on_deactivate. Cheaper and clearer than spinning a timer up and down.
  bool active_{false};

  double filtered_speed_{0.0};
  double filtered_yaw_rate_{0.0};

  static constexpr double ALPHA = 0.2;
  static constexpr double SPEED_LIMIT = 2.5;
};

} // namespace bfmc_fusion

#endif
