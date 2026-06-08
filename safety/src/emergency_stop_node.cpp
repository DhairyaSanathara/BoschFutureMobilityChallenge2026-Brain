#include "safety/emergency_stop_node.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

namespace bfmc_safety
{

EmergencyStopNode::EmergencyStopNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("emergency_stop_node", options)
{
  // CERTIFICATION FIX: do NOT initialise last_state_time_ to now() — that
  // makes the stale check trigger 1 s after launch with a false-emergency
  // (we'd be saying "state went stale" before we ever saw a state message).
  // Instead, leave last_state_time_ at the default (0,0,RCL_ROS_TIME) and
  // gate the stale check on state_ever_seen_ being true.
  imu_conf_thresh_   = declare_parameter<double>("imu_conf_thresh",   imu_conf_thresh_);
  state_conf_thresh_ = declare_parameter<double>("state_conf_thresh", state_conf_thresh_);
  state_stale_s_     = declare_parameter<double>("state_stale_s",     state_stale_s_);
  tick_period_s_     = declare_parameter<double>("tick_period_s",     tick_period_s_);
  emergency_persist_ticks_ =
      declare_parameter<int>("emergency_persist_ticks", emergency_persist_ticks_);

  auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto be  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_imu_conf_  = create_subscription<std_msgs::msg::Float64>(
    "/fusion/imu_confidence", be,
    std::bind(&EmergencyStopNode::on_imu_conf, this, std::placeholders::_1));
  sub_state_     = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", rel,
    std::bind(&EmergencyStopNode::on_state, this, std::placeholders::_1));
  sub_ready_     = create_subscription<std_msgs::msg::Bool>(
    "/system/vehicle_ready", rel,
    std::bind(&EmergencyStopNode::on_vehicle_ready, this, std::placeholders::_1));
  sub_manual_    = create_subscription<std_msgs::msg::Bool>(
    "/safety/manual_emergency", rel,
    std::bind(&EmergencyStopNode::on_manual, this, std::placeholders::_1));
  sub_watchdog_  = create_subscription<std_msgs::msg::Bool>(
    "/safety/watchdog_alarm", rel,
    std::bind(&EmergencyStopNode::on_watchdog_alarm, this, std::placeholders::_1));

  pub_soft_   = create_publisher<std_msgs::msg::Bool>("/safety/emergency",       rel);
  pub_hard_   = create_publisher<std_msgs::msg::Bool>("/safety/emergency_stop",  rel);
  pub_reason_ = create_publisher<std_msgs::msg::String>("/safety/emergency_reason", be);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(tick_period_s_ * 1000.0)),
    std::bind(&EmergencyStopNode::tick, this));

  RCLCPP_INFO(get_logger(),
    "emergency_stop_node: imu<%.2f, state<%.2f, stale>%.2fs, %0.fHz",
    imu_conf_thresh_, state_conf_thresh_, state_stale_s_, 1.0/tick_period_s_);
}

void EmergencyStopNode::on_imu_conf(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_imu_conf_.store(msg->data);
}
void EmergencyStopNode::on_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  last_state_conf_.store(msg->confidence);
  last_state_time_ = this->now();
  state_ever_seen_.store(true);
}
void EmergencyStopNode::on_vehicle_ready(const std_msgs::msg::Bool::SharedPtr msg)
{
  vehicle_ready_.store(msg->data);
}
void EmergencyStopNode::on_manual(const std_msgs::msg::Bool::SharedPtr msg)
{
  manual_.store(msg->data);
}
void EmergencyStopNode::on_watchdog_alarm(const std_msgs::msg::Bool::SharedPtr msg)
{
  watchdog_alarm_.store(msg->data);
}

void EmergencyStopNode::tick()
{
  const auto t_now = this->now();
  const bool state_seen = state_ever_seen_.load();
  const double state_age = state_seen
    ? (t_now - last_state_time_).seconds()
    : std::numeric_limits<double>::infinity();

  std::ostringstream reason;
  bool emergency = false;

  // Race-day false-emergency guard: soft sources must persist for
  // `emergency_persist_ticks_` consecutive ticks (default 5 @ 5 Hz = 1 s)
  // before they latch the emergency. Hard sources stay instant.
  auto bump_ = [&](bool bad, int & streak) -> bool {
    if (bad) {
      streak = std::min(streak + 1, emergency_persist_ticks_ + 1);
      return streak >= emergency_persist_ticks_;
    }
    streak = 0;
    return false;
  };

  // Source 1: IMU confidence collapse — persistence-gated.
  const bool imu_bad = (last_imu_conf_.load() < imu_conf_thresh_);
  if (bump_(imu_bad, imu_conf_streak_)) {
    emergency = true;
    reason << "imu_conf<" << imu_conf_thresh_
           << "(persisted " << imu_conf_streak_ << "t) ";
  }

  // Source 2 / 3: state confidence + state-stale — persistence-gated, gated
  // on having observed state at least once.
  const bool state_conf_bad =
      state_seen && last_state_conf_.load() < state_conf_thresh_;
  if (bump_(state_conf_bad, state_conf_streak_)) {
    emergency = true;
    reason << "state_conf<" << state_conf_thresh_
           << "(persisted " << state_conf_streak_ << "t) ";
  }
  const bool state_stale_bad = state_seen && state_age > state_stale_s_;
  if (bump_(state_stale_bad, state_stale_streak_)) {
    emergency = true;
    reason << "state_stale>" << state_stale_s_
           << "s(persisted " << state_stale_streak_ << "t) ";
  }

  // Source 4: system not ready (early startup gate) — INSTANT (system gate).
  if (!vehicle_ready_.load()) {
    emergency = true; reason << "not_ready ";
  }
  // Source 5: operator manual — INSTANT (red-button intent).
  if (manual_.load()) {
    emergency = true; reason << "manual ";
  }
  // Source 6: watchdog echo — INSTANT (watchdog already has its own
  // hysteresis on the upstream signal).
  if (watchdog_alarm_.load()) {
    emergency = true; reason << "watchdog ";
  }

  // Publish — latched-style via 5 Hz heartbeat.
  std_msgs::msg::Bool b; b.data = emergency;
  pub_soft_->publish(b);
  pub_hard_->publish(b);

  // Reason string only when state changes — every tick would spam.
  if (emergency != last_published_emergency_) {
    last_published_emergency_ = emergency;
    std_msgs::msg::String r;
    r.data = emergency ? reason.str() : std::string("clear");
    pub_reason_->publish(r);
    if (emergency) {
      RCLCPP_WARN(get_logger(), "EMERGENCY ASSERTED: %s", reason.str().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "EMERGENCY CLEARED");
    }
  }
}

}  // namespace bfmc_safety

// Standalone main.
#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bfmc_safety::EmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
