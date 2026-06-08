#include "imu_driver/imu_driver_node.hpp"

namespace imu_driver
{

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions & options)
: Node("imu_driver_node", options)
{
  // BEST_EFFORT, KeepLast(10), Volatile — matches every other high-rate sensor
  // sub/pub in the stack so RELIABLE/BE QoS mismatches are impossible.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  pub_ = this->create_publisher<sensor_msgs::msg::Imu>(OUTPUT_TOPIC, qos);

  sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    MICRO_ROS_TOPIC, qos,
    std::bind(&ImuDriverNode::on_micro_ros_imu, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "IMU driver started | %s → %s | transparent pass-through (no watchdog, "
    "no synthetic samples, STM32 covariances forwarded unchanged)",
    MICRO_ROS_TOPIC, OUTPUT_TOPIC);
}

void ImuDriverNode::on_micro_ros_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Transparent pass-through. The only fields we own:
  //   • header.frame_id — we anchor it to "imu_link" so downstream TF works
  //     consistently regardless of the publisher's frame_id choice.
  // Everything else (timestamp + every payload field + every covariance) is
  // forwarded verbatim from the STM32-fused message.
  sensor_msgs::msg::Imu out;
  out.header.stamp    = msg->header.stamp;
  out.header.frame_id = "imu_link";

  out.orientation         = msg->orientation;
  out.angular_velocity    = msg->angular_velocity;
  out.linear_acceleration = msg->linear_acceleration;

  out.orientation_covariance         = msg->orientation_covariance;
  out.angular_velocity_covariance    = msg->angular_velocity_covariance;
  out.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  pub_->publish(out);
}

}  // namespace imu_driver
