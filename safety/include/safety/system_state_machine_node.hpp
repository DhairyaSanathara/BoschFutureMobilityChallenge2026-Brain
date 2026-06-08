#ifndef SAFETY__SYSTEM_STATE_MACHINE_NODE_HPP_
#define SAFETY__SYSTEM_STATE_MACHINE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/path.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/object_array.hpp"
#include "bfmc_msgs/msg/trajectory.hpp"
#include "bfmc_msgs/msg/control_command.hpp"

namespace bfmc_safety
{

// Phase B + Phase Final — Startup-readiness state machine.
//
// Watches the activation chain end-to-end:
//
//   POWER_ON
//   → STM32_READY         (/microros/imu_raw observed — STM32 firmware + agent alive)
//   → MICROROS_READY      (filtered IMU observed — micro-ROS → C++ driver propagated)
//   → CAMERA_READY        (raw object detections observed — TRT engine + camera alive)
//   → IMU_READY           (legacy alias for MICROROS_READY — kept for back-compat)
//   → FUSION_READY        (/vehicle/state observed — EKF publishing)
//   → LOCALIZATION_READY  (state.confidence ≥ thresh — INFORMATIONAL ONLY)
//   → GLOBAL_PATH_READY   (/planning/global_path observed)
//   → TRAJECTORY_READY    (/planning/trajectory observed)
//   → CONTROLLER_READY    (/control/command observed)
//   → READY_TO_DRIVE
//
// LOCALIZATION_READY is informational per the Phase Final certification rule
// "any sort of localization issue should still keep the car running" — the
// chain progresses past it even if confidence drops.
//
// Outputs:
//   /system/vehicle_ready  (Bool, 2 Hz republish) — consumed by emergency_stop_node
//                                                  AND safety_supervisor_node as a
//                                                  hard gate. Until true: zeros.
//   /system/system_state   (String, 2 Hz republish) — current named state.
class SystemStateMachineNode : public rclcpp::Node
{
public:
  explicit SystemStateMachineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State : std::uint8_t {
    POWER_ON           = 0,
    STM32_READY        = 1,   // /microros/imu_raw seen
    MICROROS_READY     = 2,   // /imu/data_filtered seen (driver propagated)
    CAMERA_READY       = 3,   // /perception/objects seen (TRT detector alive)
    IMU_READY          = 4,   // alias — kept for log compat; same gate as MICROROS_READY
    FUSION_READY       = 5,   // /vehicle/state seen
    LOCALIZATION_READY = 6,   // state.confidence ≥ thresh (informational)
    GLOBAL_PATH_READY  = 7,   // /planning/global_path seen
    TRAJECTORY_READY   = 8,   // /planning/trajectory seen
    CONTROLLER_READY   = 9,   // /control/command seen
    READY_TO_DRIVE     = 10,
  };
  static const char * state_to_str(State s);

  struct Seen {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    bool         ever{false};
    bool         fresh(const rclcpp::Time & now, double stale_s) const;
  };

  // Callbacks.
  void on_microros_imu(const sensor_msgs::msg::Imu::SharedPtr);
  void on_imu         (const sensor_msgs::msg::Imu::SharedPtr);
  void on_objects     (const bfmc_msgs::msg::ObjectArray::SharedPtr);
  void on_state       (const bfmc_msgs::msg::VehicleState::SharedPtr);
  void on_path        (const nav_msgs::msg::Path::SharedPtr);
  void on_traj        (const bfmc_msgs::msg::Trajectory::SharedPtr);
  void on_command     (const bfmc_msgs::msg::ControlCommand::SharedPtr);

  void tick();
  State compute_state_(double now_s) const;

  // Subs.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           sub_microros_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           sub_imu_;
  rclcpp::Subscription<bfmc_msgs::msg::ObjectArray>::SharedPtr     sub_objects_;
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr    sub_state_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             sub_path_;
  rclcpp::Subscription<bfmc_msgs::msg::Trajectory>::SharedPtr      sub_traj_;
  rclcpp::Subscription<bfmc_msgs::msg::ControlCommand>::SharedPtr  sub_command_;

  // Pubs.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   pub_ready_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Cache.
  mutable std::mutex mtx_;
  Seen   seen_microros_imu_, seen_imu_, seen_objects_, seen_state_,
         seen_path_, seen_traj_, seen_command_;
  double last_state_confidence_{0.0};

  // Params.
  double tick_period_s_   {0.5};   // 2 Hz
  double stale_microros_s_{0.5};
  double stale_imu_s_     {0.5};
  double stale_objects_s_ {2.0};   // detector can run lazy in static scenes
  double stale_state_s_   {0.5};
  double stale_path_s_    {5.0};
  double stale_traj_s_    {1.0};
  double stale_cmd_s_     {1.0};
  double loc_min_confidence_{0.20};

  State last_state_published_{State::POWER_ON};

  // READY oscillation guard (race-day P2).
  //   Enters READY only after `ready_enter_samples_` consecutive valid ticks.
  //   Exits READY only after `ready_exit_samples_`  consecutive invalid ticks.
  //   At 2 Hz default: 20 in / 50 out → 10 s to assert, 25 s to drop.
  int  ready_enter_samples_{20};
  int  ready_exit_samples_ {50};
  int  ready_valid_streak_  {0};
  int  ready_invalid_streak_{0};
  bool ready_latched_       {false};
};

}  // namespace bfmc_safety

#endif  // SAFETY__SYSTEM_STATE_MACHINE_NODE_HPP_
