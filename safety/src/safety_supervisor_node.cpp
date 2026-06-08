#include "safety/safety_supervisor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

namespace safety
{

SafetySupervisorNode::SafetySupervisorNode(const rclcpp::NodeOptions & options)
: Node("safety_supervisor_node", options)
{
  last_cmd_.steering_angle = 0.0;
  last_cmd_.throttle       = 0.0;
  last_cmd_.brake          = 0.0;

  // Parameters — keep declarations narrow; everything else stays at the hpp default.
  forward_max_  = this->declare_parameter<double>("forward_max",  forward_max_);
  reverse_max_  = this->declare_parameter<double>("reverse_max",  reverse_max_);
  deadband_     = this->declare_parameter<double>("deadband",     deadband_);
  max_brake_    = this->declare_parameter<double>("max_brake",    max_brake_);
  heartbeat_hz_ = this->declare_parameter<double>("heartbeat_hz", heartbeat_hz_);
  cmd_stale_s_  = this->declare_parameter<double>("cmd_stale_s",  cmd_stale_s_);

  auto reliable_qos = rclcpp::QoS(10).reliable();
  auto be_qos       = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_cmd_ = this->create_subscription<bfmc_msgs::msg::ControlCommand>(
      "/control/command_raw", reliable_qos,
      std::bind(&SafetySupervisorNode::cb_command, this, std::placeholders::_1));

  sub_estop_ = this->create_subscription<std_msgs::msg::Bool>(
      "/safety/emergency_stop", reliable_qos,
      std::bind(&SafetySupervisorNode::cb_estop, this, std::placeholders::_1));

  sub_ready_ = this->create_subscription<std_msgs::msg::Bool>(
      "/system/vehicle_ready", reliable_qos,
      std::bind(&SafetySupervisorNode::cb_ready, this, std::placeholders::_1));

  pub_cmd_         = this->create_publisher<bfmc_msgs::msg::ControlCommand>(
      "/control/command", reliable_qos);
  pub_dbg_raw_     = this->create_publisher<std_msgs::msg::Float64>(
      "/control/debug/raw_throttle",     be_qos);
  pub_dbg_clamped_ = this->create_publisher<std_msgs::msg::Float64>(
      "/control/debug/clamped_throttle", be_qos);
  pub_dbg_state_   = this->create_publisher<std_msgs::msg::String>(
      "/control/debug/supervisor_state", be_qos);

  const auto period = std::chrono::microseconds(
      static_cast<int64_t>(1'000'000.0 / heartbeat_hz_));
  heartbeat_timer_ = this->create_wall_timer(
      period, std::bind(&SafetySupervisorNode::emit_safe_command, this));

  RCLCPP_INFO(this->get_logger(),
      "[SafetySupervisor] forward_max=%.2f reverse_max=%.2f deadband=%.3f hb=%.0fHz stale=%.2fs",
      forward_max_, reverse_max_, deadband_, heartbeat_hz_, cmd_stale_s_);
}

void SafetySupervisorNode::cb_command(const bfmc_msgs::msg::ControlCommand::SharedPtr msg)
{
  last_cmd_      = *msg;
  last_cmd_time_ = this->now();
}

void SafetySupervisorNode::cb_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  watchdog_estop_ = msg->data;
}

void SafetySupervisorNode::cb_ready(const std_msgs::msg::Bool::SharedPtr msg)
{
  vehicle_ready_ = msg->data;
}

void SafetySupervisorNode::emit_safe_command()
{
  // 1) Build the candidate from the latest upstream command (zero if none yet
  //    or if the upstream went stale — controller silence MUST coast to zero,
  //    never re-fire an old motion).
  bfmc_msgs::msg::ControlCommand cmd;
  cmd.steering_angle = 0.0;
  cmd.throttle       = 0.0;
  cmd.brake          = 0.0;

  const bool have_cmd =
      last_cmd_time_.nanoseconds() != 0 &&
      (this->now() - last_cmd_time_).seconds() < cmd_stale_s_;

  if (have_cmd) {
    cmd = last_cmd_;
  }

  const double raw_throttle = cmd.throttle;

  // 2) Startup gate. Defaults false; goes true only after system_state_machine
  //    asserts /system/vehicle_ready. While false: emit hard zeros regardless.
  // 3) Emergency gate. Per Phase Final P2: steering=0, throttle=0, brake=0.
  //    No reverse, no defined motion. ESC sees neutral (STM32 firmware maps
  //    all-zero ⇒ 1500 µs PWM on both servo and ESC).
  const bool block_for_ready     = !vehicle_ready_;
  const bool block_for_emergency = watchdog_estop_;
  const bool block_for_stale     = !have_cmd;

  if (block_for_ready || block_for_emergency || block_for_stale) {
    cmd.steering_angle = 0.0;
    cmd.throttle       = 0.0;
    cmd.brake          = 0.0;
  }

  // 4) Hard actuator safety — clamp + deadband. Applied last so even a
  //    runaway upstream cannot bypass the physical envelope. ControlCommand
  //    fields are float32 — convert through double to keep arithmetic precise,
  //    then narrow on assignment.
  double thr = static_cast<double>(cmd.throttle);
  thr = std::clamp(thr, -reverse_max_, forward_max_);
  if (std::abs(thr) < deadband_) {
    thr = 0.0;
  }
  cmd.throttle = static_cast<float>(thr);
  // Brake is non-negative on the contract; clamp defensively to [0, max_brake_]
  // — race-day R4: hard ceiling well below 1.0 because the STM32 firmware /
  // Quicrun ESC bench-test mapped brake=1.0 to full reverse PWM.
  double brk = std::clamp(static_cast<double>(cmd.brake), 0.0, max_brake_);
  cmd.brake = static_cast<float>(brk);

  pub_cmd_->publish(cmd);

  // 5) Debug topics.
  std_msgs::msg::Float64 d_raw;     d_raw.data     = raw_throttle;    pub_dbg_raw_->publish(d_raw);
  std_msgs::msg::Float64 d_clamped; d_clamped.data = cmd.throttle;    pub_dbg_clamped_->publish(d_clamped);

  // Only log/announce state transitions, not every tick.
  const char * reason =
      block_for_ready     ? "GATE_NOT_READY" :
      block_for_emergency ? "EMERGENCY"      :
      block_for_stale     ? "STALE_UPSTREAM" : "PASS";
  if (reason != last_emit_reason_) {
    std_msgs::msg::String s; s.data = reason; pub_dbg_state_->publish(s);
    RCLCPP_WARN(this->get_logger(),
        "[SafetySupervisor] %s → %s (raw_throttle=%.3f clamped=%.3f)",
        last_emit_reason_.c_str(), reason, raw_throttle, cmd.throttle);
    last_emit_reason_ = reason;
  }
}

}  // namespace safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety::SafetySupervisorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
