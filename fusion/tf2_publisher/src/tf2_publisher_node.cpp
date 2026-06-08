#include "tf2_publisher/tf2_publisher_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace bfmc_fusion
{

Tf2PublisherNode::Tf2PublisherNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("tf2_publisher_node", options)
{
  // Phase 8: only odom→base_link is owned by this dynamic broadcaster.
  // map→odom + base_link→camera_link + base_link→imu_link are static and
  // published by the static_transform_publisher invocations in the launch.
  odom_to_base_link_.header.frame_id = "odom";
  odom_to_base_link_.child_frame_id  = "base_link";
}

CallbackReturn Tf2PublisherNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  state_sub_ =
      create_subscription<bfmc_msgs::msg::VehicleState>(
          "/vehicle/state",
          rclcpp::QoS(5).reliable(),
          std::bind(&Tf2PublisherNode::state_callback,
                    this,std::placeholders::_1));

  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Tf2PublisherNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn Tf2PublisherNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn Tf2PublisherNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  state_sub_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn Tf2PublisherNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return CallbackReturn::SUCCESS;
}

void Tf2PublisherNode::state_callback(
  const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  // Callback-driven: stamp + broadcast in the same call. The TF age at the
  // consumer is therefore (vehicle_state publish → tf consume) plus the cost
  // of one quaternion conversion and two sendTransform calls.
  if (!active_) return;

  const rclcpp::Time stamp = now();

  // Phase 8: this node now broadcasts ONLY odom→base_link. The static
  // map→odom (identity for now), base_link→camera_link and base_link→imu_link
  // transforms are published by static_transform_publisher in the launch
  // file — single broadcaster per edge, no loops, no duplicates.
  odom_to_base_link_.header.stamp = stamp;

  odom_to_base_link_.transform.translation.x =
      msg->pose.position.x;
  odom_to_base_link_.transform.translation.y =
      msg->pose.position.y;
  odom_to_base_link_.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, msg->yaw);

  odom_to_base_link_.transform.rotation.x = q.x();
  odom_to_base_link_.transform.rotation.y = q.y();
  odom_to_base_link_.transform.rotation.z = q.z();
  odom_to_base_link_.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(odom_to_base_link_);
}

} // namespace bfmc_fusion
