#ifndef SAFETY__EMERGENCY_STOP_NODE_HPP_
#define SAFETY__EMERGENCY_STOP_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"

namespace bfmc_safety
{

// Phase 7 — OR aggregator for race-day emergency triggers.
//
// Sources (any one true ⇒ emergency = true):
//   1. /fusion/imu_confidence  < imu_conf_threshold
//   2. /vehicle/state.confidence < state_conf_threshold
//   3. /vehicle/state stale > state_stale_s
//   4. /system/vehicle_ready == false (system not ready)
//   5. /safety/manual_emergency == true (operator's red button)
//   6. /safety/watchdog_alarm == true (echo from watchdog_node, optional)
//
// Outputs (BOTH latched-style for late subscribers via 5 Hz republish):
//   /safety/emergency       (Bool)  — consumed by behavior_planner_cpp's FSM
//                                     (FsmInputs.emergency)
//   /safety/emergency_stop  (Bool)  — same topic the watchdog uses; the
//                                     trajectory_controller already brakes
//                                     hard on this. We are an ADDITIONAL
//                                     publisher; the watchdog still exists.
//
// What this node intentionally does NOT do:
//   - Generate emergencies from "stale lane" or "tracker quiet" — those are
//     normal degraded states that the FSM handles via DetectionTimer
//     hysteresis and target_speed gating. Hard e-stop reserves for sensor
//     loss / system unreadiness / explicit operator command.
class EmergencyStopNode : public rclcpp::Node
{
public:
  explicit EmergencyStopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_imu_conf      (const std_msgs::msg::Float64::SharedPtr msg);
  void on_state         (const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void on_vehicle_ready (const std_msgs::msg::Bool::SharedPtr msg);
  void on_manual        (const std_msgs::msg::Bool::SharedPtr msg);
  void on_watchdog_alarm(const std_msgs::msg::Bool::SharedPtr msg);
  void tick();

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      sub_imu_conf_;
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_ready_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_manual_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_watchdog_;

  // /safety/emergency  is the soft "FSM, please brake" signal (latching
  // 5 Hz republish for late subscribers). /safety/emergency_stop is the
  // hard "controller, neutralize" channel — same topic the watchdog uses,
  // so we co-publish (ROS2 allows multiple publishers on Reliable topics).
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_soft_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_hard_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_reason_;
  rclcpp::TimerBase::SharedPtr                                 timer_;

  // Cached source state.
  std::atomic<double> last_imu_conf_{1.0};
  std::atomic<bool>   manual_{false};
  std::atomic<bool>   watchdog_alarm_{false};
  // vehicle_ready_ defaults FALSE so cold-start (no /system/vehicle_ready yet)
  // keeps the car held. system_state_machine flips it true when chain green.
  std::atomic<bool>   vehicle_ready_{false};
  std::atomic<double> last_state_conf_{1.0};
  rclcpp::Time        last_state_time_{0,0,RCL_ROS_TIME};
  // Has /vehicle/state ever been observed? Used to suppress the state-stale
  // emergency during cold-start (before fusion comes online).
  std::atomic<bool>   state_ever_seen_{false};

  // Tunables (ROS parameters).
  double imu_conf_thresh_  {0.25};
  double state_conf_thresh_{0.15};
  // CERTIFICATION FIX: relaxed 0.3 → 1.0 s. The behavior_planner already
  // pivots to EMERGENCY_STOP at 0.2 s stale (soft brake via target_speed=0);
  // this hard-stop layer should only fire on a sustained loss. 1.0 s gives
  // the soft layer time to act and avoids false-emergencies on brief stutters.
  double state_stale_s_    {1.0};
  double tick_period_s_    {0.20};   // 5 Hz republish

  // Race-day false-emergency guard. Soft sources (imu_conf / state_conf /
  // state_stale) must be true for N consecutive ticks before they latch the
  // emergency. Hard sources (manual button, watchdog alarm, vehicle_ready=
  // false) are instant — they represent operator / system-level overrides.
  // At 5 Hz tick → 5 ticks = 1.0 s consecutive bad signal before STOP.
  int  emergency_persist_ticks_{5};
  int  imu_conf_streak_  {0};
  int  state_conf_streak_{0};
  int  state_stale_streak_{0};

  // Last published state (so we only LOG on transitions, not every tick).
  bool last_published_emergency_{false};
};

}  // namespace bfmc_safety

#endif  // SAFETY__EMERGENCY_STOP_NODE_HPP_
