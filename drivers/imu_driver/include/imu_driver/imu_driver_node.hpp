#ifndef IMU_DRIVER__IMU_DRIVER_NODE_HPP_
#define IMU_DRIVER__IMU_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imu_driver
{

// Transparent pass-through (Phase 2.7).
//
// STM32 already runs onboard sensor fusion (gyro + accel + mag, magnetometer
// enabled, chassis-pitch calibrated) and ships a fully-formed sensor_msgs/Imu
// message on /microros/imu_raw. This node:
//
//   • re-publishes the message on /imu/data_filtered with frame_id="imu_link"
//   • preserves the STM32 hardware timestamp
//   • forwards orientation / angular_velocity / linear_acceleration unchanged
//   • forwards all three covariance matrices unchanged
//
// There is no watchdog, no synthetic-sample fallback, no covariance overwrite.
// On micro-ROS link loss /imu/data_filtered simply goes silent and downstream
// staleness gates (EKF's IMU_TIMEOUT, health_monitor's stale_imu_count) take
// over. That is intentional: fabricating zero gyro/accel/orientation samples
// would feed the Kalman filter a confident "stationary, north-facing"
// measurement and corrupt the state.
class ImuDriverNode : public rclcpp::Node
{
public:
  explicit ImuDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuDriverNode() override = default;

private:
  void on_micro_ros_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  static constexpr const char * OUTPUT_TOPIC    = "/imu/data_filtered";
  static constexpr const char * MICRO_ROS_TOPIC = "/microros/imu_raw";
};

}  // namespace imu_driver

#endif  // IMU_DRIVER__IMU_DRIVER_NODE_HPP_
