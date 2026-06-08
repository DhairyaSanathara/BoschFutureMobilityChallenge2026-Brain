#include "safety/system_state_machine_node.hpp"

#include <algorithm>
#include <chrono>

namespace bfmc_safety
{

const char * SystemStateMachineNode::state_to_str(State s)
{
  switch (s) {
    case State::POWER_ON:           return "POWER_ON";
    case State::STM32_READY:        return "STM32_READY";
    case State::MICROROS_READY:     return "MICROROS_READY";
    case State::CAMERA_READY:       return "CAMERA_READY";
    case State::IMU_READY:          return "IMU_READY";
    case State::FUSION_READY:       return "FUSION_READY";
    case State::LOCALIZATION_READY: return "LOCALIZATION_READY";
    case State::GLOBAL_PATH_READY:  return "GLOBAL_PATH_READY";
    case State::TRAJECTORY_READY:   return "TRAJECTORY_READY";
    case State::CONTROLLER_READY:   return "CONTROLLER_READY";
    case State::READY_TO_DRIVE:     return "READY_TO_DRIVE";
  }
  return "UNKNOWN";
}

bool SystemStateMachineNode::Seen::fresh(const rclcpp::Time & now, double stale_s) const
{
  return ever && (now - t).seconds() < stale_s;
}

SystemStateMachineNode::SystemStateMachineNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("system_state_machine_node", options)
{
  tick_period_s_      = declare_parameter<double>("tick_period_s",      tick_period_s_);
  stale_microros_s_   = declare_parameter<double>("stale_microros_s",   stale_microros_s_);
  stale_imu_s_        = declare_parameter<double>("stale_imu_s",        stale_imu_s_);
  stale_objects_s_    = declare_parameter<double>("stale_objects_s",    stale_objects_s_);
  stale_state_s_      = declare_parameter<double>("stale_state_s",      stale_state_s_);
  stale_path_s_       = declare_parameter<double>("stale_path_s",       stale_path_s_);
  stale_traj_s_       = declare_parameter<double>("stale_traj_s",       stale_traj_s_);
  stale_cmd_s_        = declare_parameter<double>("stale_cmd_s",        stale_cmd_s_);
  loc_min_confidence_ = declare_parameter<double>("loc_min_confidence", loc_min_confidence_);
  ready_enter_samples_ = declare_parameter<int>("ready_enter_samples", ready_enter_samples_);
  ready_exit_samples_  = declare_parameter<int>("ready_exit_samples",  ready_exit_samples_);

  auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto be  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  // /microros/imu_raw — micro-ROS publishes BE (typical), so subscribe BE.
  sub_microros_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/microros/imu_raw", be,
    std::bind(&SystemStateMachineNode::on_microros_imu, this, std::placeholders::_1));
  // /imu/data_filtered — imu_driver publishes BestEffort KeepLast(10).
  sub_imu_     = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_filtered", be,
    std::bind(&SystemStateMachineNode::on_imu, this, std::placeholders::_1));
  // /perception/objects — object_detection_cpp publishes Reliable.
  sub_objects_ = create_subscription<bfmc_msgs::msg::ObjectArray>(
    "/perception/objects", rel,
    std::bind(&SystemStateMachineNode::on_objects, this, std::placeholders::_1));
  // /vehicle/state — vehicle_state_node publishes Reliable.
  sub_state_   = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", rel,
    std::bind(&SystemStateMachineNode::on_state, this, std::placeholders::_1));
  // /planning/global_path — global_path_planner publishes Reliable.
  sub_path_    = create_subscription<nav_msgs::msg::Path>(
    "/planning/global_path", rel,
    std::bind(&SystemStateMachineNode::on_path, this, std::placeholders::_1));
  // /planning/trajectory — trajectory_generator publishes BestEffort
  // (BE_QoS(1)). Phase Final fix: must subscribe BestEffort or the contract
  // mismatches and TRAJECTORY_READY never asserts → vehicle_ready locked false.
  sub_traj_    = create_subscription<bfmc_msgs::msg::Trajectory>(
    "/planning/trajectory", be,
    std::bind(&SystemStateMachineNode::on_traj, this, std::placeholders::_1));
  // /control/command — safety_supervisor publishes Reliable.
  sub_command_ = create_subscription<bfmc_msgs::msg::ControlCommand>(
    "/control/command", rel,
    std::bind(&SystemStateMachineNode::on_command, this, std::placeholders::_1));

  pub_ready_ = create_publisher<std_msgs::msg::Bool>("/system/vehicle_ready", rel);
  pub_state_ = create_publisher<std_msgs::msg::String>("/system/system_state", rel);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(tick_period_s_ * 1000.0)),
    std::bind(&SystemStateMachineNode::tick, this));

  RCLCPP_INFO(get_logger(),
    "system_state_machine: %.0f Hz tick, stale[microros=%.1f imu=%.1f objects=%.1f "
    "state=%.1f path=%.1f traj=%.1f cmd=%.1f]",
    1.0/tick_period_s_, stale_microros_s_, stale_imu_s_, stale_objects_s_,
    stale_state_s_, stale_path_s_, stale_traj_s_, stale_cmd_s_);
}

void SystemStateMachineNode::on_microros_imu(const sensor_msgs::msg::Imu::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_microros_imu_.t = this->now(); seen_microros_imu_.ever = true;
}
void SystemStateMachineNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_imu_.t = this->now(); seen_imu_.ever = true;
}
void SystemStateMachineNode::on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_objects_.t = this->now(); seen_objects_.ever = true;
}
void SystemStateMachineNode::on_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_state_.t = this->now(); seen_state_.ever = true;
  last_state_confidence_ = msg->confidence;
}
void SystemStateMachineNode::on_path(const nav_msgs::msg::Path::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_path_.t = this->now(); seen_path_.ever = true;
}
void SystemStateMachineNode::on_traj(const bfmc_msgs::msg::Trajectory::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_traj_.t = this->now(); seen_traj_.ever = true;
}
void SystemStateMachineNode::on_command(const bfmc_msgs::msg::ControlCommand::SharedPtr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  seen_command_.t = this->now(); seen_command_.ever = true;
}

SystemStateMachineNode::State
SystemStateMachineNode::compute_state_(double /*now_s*/) const
{
  // Lock-protected snapshot from caller. Sequential gating.
  const auto t_now = this->now();
  if (!seen_microros_imu_.fresh(t_now, stale_microros_s_)) return State::POWER_ON;
  if (!seen_imu_         .fresh(t_now, stale_imu_s_))      return State::STM32_READY;
  if (!seen_objects_     .fresh(t_now, stale_objects_s_))  return State::MICROROS_READY;
  if (!seen_state_       .fresh(t_now, stale_state_s_))    return State::CAMERA_READY;
  // CERTIFICATION FIX: LOCALIZATION_READY is INFORMATIONAL — the user's race-day
  // rule is "any localization issue should still keep the car running". As long
  // as the EKF is publishing /vehicle/state (FUSION_READY), the chain proceeds.
  // If localization actually drops out at runtime, state.confidence settles
  // around 0.6 (IMU + cov health survive), which is above the behavior_planner's
  // 0.20 EMERGENCY_STOP gate — the FSM degrades into perception-only LANE_KEEP.
  const bool loc_ok = (last_state_confidence_ >= loc_min_confidence_);
  if (!seen_path_   .fresh(t_now, stale_path_s_)) {
    return loc_ok ? State::LOCALIZATION_READY : State::FUSION_READY;
  }
  if (!seen_traj_   .fresh(t_now, stale_traj_s_))    return State::GLOBAL_PATH_READY;
  if (!seen_command_.fresh(t_now, stale_cmd_s_))     return State::TRAJECTORY_READY;
  return State::READY_TO_DRIVE;
}

void SystemStateMachineNode::tick()
{
  State s;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    s = compute_state_(this->now().seconds());
  }

  // Hysteresis on the published ready flag: require N consecutive valid ticks
  // to assert, M consecutive invalid ticks to drop. Prevents READY↔NOT_READY
  // oscillation when one upstream topic occasionally misses its stale window.
  const bool instantaneous_ready = (s == State::READY_TO_DRIVE);
  if (instantaneous_ready) {
    ready_valid_streak_   = std::min(ready_valid_streak_ + 1, ready_enter_samples_ + 1);
    ready_invalid_streak_ = 0;
  } else {
    ready_invalid_streak_ = std::min(ready_invalid_streak_ + 1, ready_exit_samples_ + 1);
    ready_valid_streak_   = 0;
  }
  const bool was_latched = ready_latched_;
  if (!ready_latched_ && ready_valid_streak_   >= ready_enter_samples_) ready_latched_ = true;
  if ( ready_latched_ && ready_invalid_streak_ >= ready_exit_samples_)  ready_latched_ = false;
  if (was_latched != ready_latched_) {
    RCLCPP_INFO(get_logger(),
      "system_state: ready latch %s (state=%s, valid_streak=%d, invalid_streak=%d)",
      ready_latched_ ? "ENGAGED" : "RELEASED",
      state_to_str(s), ready_valid_streak_, ready_invalid_streak_);
  }

  std_msgs::msg::Bool ready;
  ready.data = ready_latched_;
  pub_ready_->publish(ready);

  std_msgs::msg::String ss;
  ss.data = state_to_str(s);
  pub_state_->publish(ss);

  if (s != last_state_published_) {
    RCLCPP_INFO(get_logger(),
      "system_state: %s → %s (latched_ready=%s)",
      state_to_str(last_state_published_), state_to_str(s),
      ready_latched_ ? "true" : "false");
    last_state_published_ = s;
  }
}

}  // namespace bfmc_safety

#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bfmc_safety::SystemStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
