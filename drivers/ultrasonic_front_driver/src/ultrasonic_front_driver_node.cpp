#include "ultrasonic_front_driver/ultrasonic_front_driver_node.hpp"

namespace ultrasonic_front_driver
{

UltrasonicFrontNode::UltrasonicFrontNode(const rclcpp::NodeOptions & options)
: Node("ultrasonic_front_driver_node", options)
{
  // Initialize connection state and timestamp
  micro_ros_connected_ = false;
  last_msg_time_ = this->now();

  // QoS: BEST_EFFORT for sensor data (matches micro-ROS default)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  pub_ = this->create_publisher<std_msgs::msg::Float32>(OUTPUT_TOPIC, qos);

  // Preallocate
  msg_.data = INVALID_READING;

  // Subscribe to micro-ROS range expecting sensor_msgs::msg::Range
  sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    MICRO_ROS_TOPIC, qos,
    std::bind(&UltrasonicFrontNode::on_micro_ros_range, this, std::placeholders::_1));

  // Fallback timer
  auto period = std::chrono::duration<double>(1.0 / PUBLISH_RATE_HZ);
  fallback_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UltrasonicFrontNode::on_fallback_timer, this));

  RCLCPP_INFO(this->get_logger(),
    "Ultrasonic front driver started | output=%s | rate=%.0f Hz",
    OUTPUT_TOPIC, PUBLISH_RATE_HZ);
}

float UltrasonicFrontNode::clamp_distance(float raw)
{
  if (std::isnan(raw) || std::isinf(raw)) {
    return INVALID_READING;
  }
  if (raw < MIN_RANGE_M || raw > MAX_RANGE_M) {
    return INVALID_READING;
  }
  return raw;
}

void UltrasonicFrontNode::on_micro_ros_range(const sensor_msgs::msg::Range::SharedPtr msg)
{
  micro_ros_connected_ = true;
  last_msg_time_ = this->now(); // Update timestamp on every received message
  
  // Extract the actual float value from the Range message struct
  msg_.data = clamp_distance(msg->range);
  pub_->publish(msg_);
}

void UltrasonicFrontNode::on_fallback_timer()
{
  // Timeout Failsafe: If connected but no message in 0.2 seconds, connection dropped!
  if (micro_ros_connected_ && (this->now() - last_msg_time_).seconds() < 0.2) {
    return; // Data is flowing normally
  }

  // Connection lost or hasn't started yet. Publish safe fallback data.
  msg_.data = INVALID_READING;
  pub_->publish(msg_);
}

}  // namespace ultrasonic_front_driver
