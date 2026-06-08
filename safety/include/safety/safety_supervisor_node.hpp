#ifndef SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
#define SAFETY__SAFETY_SUPERVISOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "bfmc_msgs/msg/control_command.hpp"

namespace safety
{

// Final control-command gate. Sits between the trajectory_controller
// (/control/command_raw) and the micro-ROS bridge (/control/command). Three
// race-day responsibilities, applied in this order:
//
//   1. Startup gate — until /system/vehicle_ready is true, output ALL ZEROS.
//   2. Emergency gate — when /safety/emergency_stop is true, output ALL ZEROS
//      (no reverse, no brake spike — just neutral, per Phase Final P2).
//   3. Hard actuator safety — clamp final throttle to [-forward_max,+reverse_max]
//      with a |throttle| < deadband zero-band so the ESC never sees creep.
//
// A 50 Hz heartbeat timer guarantees the bridge always sees a fresh frame
// (zeros if upstream is silent), so the STM32 cannot latch a stale command.
class SafetySupervisorNode : public rclcpp::Node
{
public:
  explicit SafetySupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Subscriptions / publishers.
  rclcpp::Subscription<bfmc_msgs::msg::ControlCommand>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            sub_estop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            sub_ready_;
  rclcpp::Publisher<bfmc_msgs::msg::ControlCommand>::SharedPtr    pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            pub_dbg_raw_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            pub_dbg_clamped_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pub_dbg_state_;

  // Heartbeat timer — runs at heartbeat_hz_ to guarantee fresh output frames.
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // State (atomic-safe single-threaded since rclcpp::spin uses one executor).
  bool watchdog_estop_{false};
  // vehicle_ready_ defaults FALSE so cold-start (no system_state_machine pub yet)
  // keeps the car held. system_state_machine flips it true when chain green.
  bool vehicle_ready_{false};
  bfmc_msgs::msg::ControlCommand last_cmd_;   // upstream raw command
  rclcpp::Time                    last_cmd_time_{0,0,RCL_ROS_TIME};

  // Tunables.
  double forward_max_     {0.35};   // hard throttle clamp upper bound
  // Race-day P3: reverse motion physically disallowed at this layer.
  // Clamp range is [-reverse_max_, +forward_max_]. Default 0 → no negative
  // throttle escapes the supervisor regardless of upstream. Override via
  // param only if a reverse maneuver is deliberately authorized.
  double reverse_max_     {0.0};
  double deadband_        {0.05};   // |throttle| < this → 0 (kills ESC creep)
  // Race-day R4: brake hard ceiling. Bench-observed firmware bug: Quicrun
  // Fusion ESC + STM32 firmware interpret brake=1.0 as "full reverse PWM"
  // rather than mechanical brake. Capping at 0.5 prevents any commanded
  // brake from saturating into reverse motion. The local planner only
  // commands brake=0.15 (gentle stop-sign brake) — far below this cap.
  double max_brake_       {0.5};
  double heartbeat_hz_    {50.0};   // republish rate (matches controller)
  double cmd_stale_s_     {0.20};   // upstream silence → fall back to zeros
  std::string last_emit_reason_{"boot"};

  // Callbacks.
  void cb_command   (const bfmc_msgs::msg::ControlCommand::SharedPtr msg);
  void cb_estop     (const std_msgs::msg::Bool::SharedPtr msg);
  void cb_ready     (const std_msgs::msg::Bool::SharedPtr msg);

  // Heartbeat: clamp + gate + publish.
  void emit_safe_command();
};

}  // namespace safety

#endif  // SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
