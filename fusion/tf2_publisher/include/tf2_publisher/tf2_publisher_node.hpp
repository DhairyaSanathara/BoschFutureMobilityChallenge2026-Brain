#ifndef TF2_PUBLISHER__TF2_PUBLISHER_NODE_HPP_
#define TF2_PUBLISHER__TF2_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <memory>

namespace bfmc_fusion
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Tf2PublisherNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Tf2PublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Callback-driven: every /vehicle/state sample is converted directly into
  // TF broadcasts in the same call. No timer phase delay.
  void state_callback(const bfmc_msgs::msg::VehicleState::SharedPtr msg);

  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr state_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Lifecycle-active gate. Subscription exists from on_configure but we must
  // not broadcast TF until on_activate, and must stop after on_deactivate.
  bool active_{false};

  // Phase 8: only odom→base_link is owned here. map→odom + base_link→*
  // are static (launch-published).
  geometry_msgs::msg::TransformStamped odom_to_base_link_;
};

} // namespace bfmc_fusion

#endif
