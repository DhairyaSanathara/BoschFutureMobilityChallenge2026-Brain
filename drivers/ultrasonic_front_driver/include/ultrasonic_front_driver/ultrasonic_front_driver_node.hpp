#ifndef ULTRASONIC_FRONT_DRIVER__ULTRASONIC_FRONT_DRIVER_NODE_HPP_
#define ULTRASONIC_FRONT_DRIVER__ULTRASONIC_FRONT_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

// 1️⃣ Added Range message include
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32.hpp>

namespace ultrasonic_front_driver
{

class UltrasonicFrontNode : public rclcpp::Node
{
public:
  explicit UltrasonicFrontNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UltrasonicFrontNode() override = default;

private:

  // 3️⃣ Updated callback parameter to Range
  void on_micro_ros_range(const sensor_msgs::msg::Range::SharedPtr msg);

  void on_fallback_timer();

  static float clamp_distance(float raw);

  // 2️⃣ Updated subscription type to Range
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr fallback_timer_;

  std_msgs::msg::Float32 msg_;

  // 4️⃣ Added timestamp tracking for timeout logic
  rclcpp::Time last_msg_time_;

  bool micro_ros_connected_;

  static constexpr double PUBLISH_RATE_HZ = 20.0;
  static constexpr float MIN_RANGE_M = 0.02f;
  static constexpr float MAX_RANGE_M = 4.00f;
  static constexpr float INVALID_READING = -1.0f;

  static constexpr const char * OUTPUT_TOPIC = "/ultrasonic/front";
  static constexpr const char * MICRO_ROS_TOPIC = "/microros/ultrasonic_front";
};

}  // namespace ultrasonic_front_driver

#endif  // ULTRASONIC_FRONT_DRIVER__ULTRASONIC_FRONT_DRIVER_NODE_HPP_
