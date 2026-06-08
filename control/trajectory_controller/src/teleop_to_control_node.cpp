// Race-day manual mode bridge: geometry_msgs/Twist → bfmc_msgs/ControlCommand.
//
// Drives /control/command directly (bypasses the autonomy supervisor) so the
// operator can drive with teleop_twist_keyboard during desk validation or
// manual test laps. Applies the SAME physical envelope the supervisor would
// (throttle ≥ 0 always, brake bounded, deadband) so the firmware can't be
// asked for negative PWM.
//
// Mapping:
//   linear.x  →  throttle (m/s scale, clamped [0, max_throttle])
//                Negative linear.x is treated as brake (no reverse motion).
//   angular.z →  steering_angle (rad, clamped [-max_steer, +max_steer])
//   max_speed_mps: anchors linear.x → throttle scale.
//
// Publishes at the same 50 Hz heartbeat as the autonomy controller would, so
// the STM32 sees a steady command stream and never falls into a stale-input
// state.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bfmc_msgs/msg/control_command.hpp"

class TeleopToControlNode : public rclcpp::Node
{
public:
  TeleopToControlNode()
  : rclcpp::Node("teleop_to_control_node")
  {
    max_throttle_  = declare_parameter<double>("max_throttle",  0.35);
    max_brake_     = declare_parameter<double>("max_brake",     0.45);
    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.436);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 0.50);
    deadband_      = declare_parameter<double>("deadband",      0.05);
    publish_hz_    = declare_parameter<double>("publish_hz",    50.0);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);

    auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&TeleopToControlNode::on_cmd_vel, this, std::placeholders::_1));
    pub_ = create_publisher<bfmc_msgs::msg::ControlCommand>("/control/command", rel);

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TeleopToControlNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "teleop_to_control: %.0f Hz, max_throttle=%.2f max_brake=%.2f "
      "max_steer=%.3f rad max_speed=%.2f m/s deadband=%.2f cmd_timeout=%.2fs",
      publish_hz_, max_throttle_, max_brake_, max_steer_rad_, max_speed_mps_,
      deadband_, cmd_timeout_s_);
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_lin_x_ = std::isfinite(msg->linear.x)  ? msg->linear.x  : 0.0;
    last_ang_z_ = std::isfinite(msg->angular.z) ? msg->angular.z : 0.0;
    last_cmd_t_ = this->now();
    have_cmd_   = true;
  }

  void tick()
  {
    bfmc_msgs::msg::ControlCommand cmd;
    cmd.steering_angle = 0.0;
    cmd.throttle       = 0.0;
    cmd.brake          = 0.0;

    // Stale timeout: if no /cmd_vel for cmd_timeout_s_, publish neutral.
    const bool stale =
        !have_cmd_ ||
        (this->now() - last_cmd_t_).seconds() > cmd_timeout_s_;
    if (stale) { pub_->publish(cmd); return; }

    // Map angular.z → steering. Positive z = left turn.
    double steer = std::clamp(last_ang_z_, -max_steer_rad_, max_steer_rad_);
    cmd.steering_angle = static_cast<float>(steer);

    // Map linear.x → throttle/brake. No reverse motion possible.
    if (last_lin_x_ > 0.0) {
      const double norm = std::clamp(last_lin_x_ / max_speed_mps_, 0.0, 1.0);
      double thr = norm * max_throttle_;
      if (std::abs(thr) < deadband_) thr = 0.0;
      cmd.throttle = static_cast<float>(thr);
    } else if (last_lin_x_ < 0.0) {
      // Teleop "reverse" press → brake, never reverse PWM.
      const double mag = std::min(-last_lin_x_ / max_speed_mps_, 1.0);
      cmd.brake = static_cast<float>(mag * max_brake_);
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<bfmc_msgs::msg::ControlCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_throttle_{0.35};
  double max_brake_{0.45};
  double max_steer_rad_{0.436};
  double max_speed_mps_{0.50};
  double deadband_{0.05};
  double publish_hz_{50.0};
  double cmd_timeout_s_{0.5};

  bool have_cmd_{false};
  rclcpp::Time last_cmd_t_{0, 0, RCL_ROS_TIME};
  double last_lin_x_{0.0};
  double last_ang_z_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopToControlNode>());
  rclcpp::shutdown();
  return 0;
}
