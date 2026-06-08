#include "vehicle_state/vehicle_state_node.hpp"
#include <algorithm>
#include <cmath>

namespace bfmc_fusion
{

VehicleStateNode::VehicleStateNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vehicle_state_node", options)
{}

CallbackReturn VehicleStateNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto qos = rclcpp::QoS(5).reliable();

  raw_sub_ = create_subscription<bfmc_msgs::msg::VehicleState>(
      "/vehicle/state_raw",
      qos,
      std::bind(&VehicleStateNode::state_raw_callback,
                this,std::placeholders::_1));

  state_pub_ = create_publisher<bfmc_msgs::msg::VehicleState>(
      "/vehicle/state",
      rclcpp::QoS(10).reliable());

  return CallbackReturn::SUCCESS;
}

CallbackReturn VehicleStateNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  state_pub_->on_activate();

  filtered_speed_ = 0.0;
  filtered_yaw_rate_ = 0.0;
  active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn VehicleStateNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VehicleStateNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  raw_sub_.reset();
  state_pub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VehicleStateNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return CallbackReturn::SUCCESS;
}

void VehicleStateNode::state_raw_callback(
  const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  // Callback-driven: filter + publish in the same call. No timer phase delay.
  if (!active_) return;

  if (!std::isfinite(msg->yaw) ||
      !std::isfinite(msg->pose.position.x) ||
      !std::isfinite(msg->pose.position.y))
    return;

  filtered_msg_ = *msg;

  const double raw_speed =
      std::isfinite(msg->speed) ? msg->speed : 0.0;
  const double raw_yaw_rate =
      std::isfinite(msg->yaw_rate) ? msg->yaw_rate : 0.0;

  filtered_speed_ =
      (1.0 - ALPHA) * filtered_speed_ +
      ALPHA * raw_speed;

  filtered_yaw_rate_ =
      (1.0 - ALPHA) * filtered_yaw_rate_ +
      ALPHA * raw_yaw_rate;

  filtered_speed_ =
      std::clamp(filtered_speed_,
                 -SPEED_LIMIT,
                 SPEED_LIMIT);

  filtered_msg_.speed = filtered_speed_;
  filtered_msg_.yaw_rate = filtered_yaw_rate_;
  filtered_msg_.twist.linear.x = filtered_speed_;
  filtered_msg_.twist.angular.z = filtered_yaw_rate_;

  state_pub_->publish(filtered_msg_);
}

} // namespace bfmc_fusion
