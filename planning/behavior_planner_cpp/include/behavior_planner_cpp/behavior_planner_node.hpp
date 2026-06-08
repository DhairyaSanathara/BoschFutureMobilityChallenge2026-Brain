#ifndef BEHAVIOR_PLANNER_CPP__BEHAVIOR_PLANNER_NODE_HPP_
#define BEHAVIOR_PLANNER_CPP__BEHAVIOR_PLANNER_NODE_HPP_

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"

#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/object_array.hpp"
#include "bfmc_msgs/msg/lane_geometry.hpp"
#include "bfmc_msgs/msg/free_space.hpp"

#include "behavior_planner_cpp/fsm.hpp"

namespace behavior_planner_cpp
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BehaviorPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BehaviorPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /* ---------------- Lifecycle ---------------- */
  CallbackReturn on_configure (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup   (const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown  (const rclcpp_lifecycle::State &) override;

  /* ---------------- Callbacks ---------------- */
  void on_state  (const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr msg);
  void on_lane   (const bfmc_msgs::msg::LaneGeometry::SharedPtr msg);
  void on_path   (const nav_msgs::msg::Path::SharedPtr msg);
  void on_imu_conf(const std_msgs::msg::Float64::SharedPtr msg);
  void on_emergency(const std_msgs::msg::Bool::SharedPtr msg);
  void on_freespace(const bfmc_msgs::msg::FreeSpace::SharedPtr msg);
  void on_nav_flag (const std_msgs::msg::String::SharedPtr msg);
  void on_semaphore(const std_msgs::msg::String::SharedPtr msg);  // /v2x/semaphores
  void on_mission_speed_limit(const std_msgs::msg::Float64::SharedPtr msg);  // mission_supervisor speed override

  /* ---------------- Timer-driven FSM tick ---------------- */
  void fsm_tick();

  // Builds the FsmInputs for this tick from cached subscriptions. Drives
  // every DetectionTimer with the latest scan of /perception/tracked_objects.
  FsmInputs build_inputs_(double now_s);

  // Intersection turn lookahead from the cached global_path. 'L' / 'R'
  // or std::nullopt for "go straight".
  std::optional<char> derive_intersection_turn_() const;

  // FreeSpace left-column occupancy check. Returns true iff < 15% of the
  // left-side cells are above the occupancy threshold (mirrors Python).
  bool freespace_left_clear_(const bfmc_msgs::msg::FreeSpace & fs,
                             int col_start, int col_end,
                             double threshold) const;

  /* ---------------- Subs ---------------- */
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Subscription<bfmc_msgs::msg::ObjectArray>::SharedPtr  sub_objects_;
  rclcpp::Subscription<bfmc_msgs::msg::LaneGeometry>::SharedPtr sub_lane_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr          sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       sub_imu_conf_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          sub_emergency_;
  rclcpp::Subscription<bfmc_msgs::msg::FreeSpace>::SharedPtr    sub_freespace_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_nav_flag_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_semaphore_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       sub_mission_speed_;

  /* ---------------- Pubs ---------------- */
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr  pub_behavior_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr pub_target_speed_;
  // Race-day overtake: lateral offset (m, positive = left) for the local
  // planner to add to every TrajectoryPoint. 0 when not overtaking.
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr pub_lateral_offset_;

  // Debug
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dbg_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dbg_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dbg_goal_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_dbg_dev_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dbg_next_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dbg_transition_;

  rclcpp::TimerBase::SharedPtr fsm_timer_;

  /* ---------------- Cached inputs (cache_mtx_) --------------- */
  mutable std::mutex cache_mtx_;
  bool   have_state_{false};
  bfmc_msgs::msg::VehicleState  latest_state_;
  bfmc_msgs::msg::ObjectArray   latest_objects_;
  bfmc_msgs::msg::LaneGeometry  latest_lane_;
  nav_msgs::msg::Path           latest_path_;
  bfmc_msgs::msg::FreeSpace     latest_freespace_;
  double last_imu_conf_{1.0};
  bool   emergency_flag_{false};
  std::string nav_flag_;          // last value of /navigation/flags
  // V2X semaphore cache: id → state ("red" / "yellow" / "green")
  std::unordered_map<std::string, std::string> semaphore_state_;
  // Mission-supervisor speed override (≤ 0 = "no override"). Cached
  // atomically — single producer / single consumer pattern.
  std::atomic<double> mission_speed_limit_{-1.0};
  // When true (default), in LANE_KEEP / FOLLOW (overtake-intent) maneuvers
  // we publish max(fsm_speed, mission_speed_limit_) so the YAML profile
  // actually drives the highway/normal sections. Safety-priority maneuvers
  // (PEDESTRIAN_WAIT, STOP, EMERGENCY_STOP, CROSSWALK_SLOW, PARK,
  // ROUNDABOUT_*, HIGHWAY_ENTER/EXIT ramps, INTERSECTION_*) keep the FSM
  // speed unchanged — they are deliberately conservative.
  bool   use_mission_speed_{true};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lane_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_objects_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_freespace_time_{0, 0, RCL_ROS_TIME};

  /* ---------------- FSM + tuning ---------------- */
  Fsm        fsm_;
  FsmTuning  tuning_;

  bool       active_{false};

  /* ---------------- DetectionTimer per object class -------------- */
  // Hysteresis on each trigger class. Per-class timers (Python uses the same
  // structure). Keys are the canonical class strings we expect from the
  // detector — `classes_*` parameter lists tell us which strings to map onto
  // which canonical key.
  std::unordered_map<std::string, DetectionTimer> det_timers_;

  /* ---------------- Parameters ---------------- */
  // Race-day P4: planner ticks at 5 Hz (200 ms) — was 20 Hz. The FSM has no
  // benefit running faster than camera-derived perception (lane 15 Hz, objects
  // 8 Hz). Param-overridable via launch.
  double tick_period_s_{0.20};
  // Class-name pattern lists — defaults now MATCH the BFMC YOLOv8 detector
  // exactly (per Phase Final audit). Override via ROS params if the detector
  // class set ever changes.
  std::vector<std::string> classes_pedestrian_    {"pedestrian"};
  std::vector<std::string> classes_traffic_light_ {"trafficlight"};
  std::vector<std::string> classes_stop_sign_     {"stop"};
  std::vector<std::string> classes_roadblock_     {"roadblock"};
  std::vector<std::string> classes_vehicle_       {"car"};
  std::vector<std::string> classes_crosswalk_     {"crosswalk"};
  std::vector<std::string> classes_parking_       {"parking"};
  std::vector<std::string> classes_roundabout_    {"roundabout"};
  std::vector<std::string> classes_highway_entry_ {"highway_entry"};
  std::vector<std::string> classes_highway_exit_  {"highway_exit"};
  // Race-day finals additions (BFMC detector class names).
  std::vector<std::string> classes_no_entry_      {"no_entry"};
  std::vector<std::string> classes_priority_      {"priority"};
  std::vector<std::string> classes_onewayroad_    {"onewayroad"};

  // Hysteresis timing per class (seconds) — matches Python defaults.
  double pedestrian_confirm_s_ {0.25};
  double pedestrian_clear_s_   {1.00};
  double vehicle_confirm_s_    {0.30};
  double vehicle_clear_s_      {0.80};
  double sign_confirm_s_       {0.20};
  double sign_clear_s_         {0.60};
  double roadblock_confirm_s_  {0.50};
  double roadblock_clear_s_    {1.50};

  // Stale gates.
  double stale_state_s_     {0.20};
  double stale_lane_s_      {0.50};
  double stale_objects_s_   {0.50};
  double stale_path_s_      {5.00};
  double stale_freespace_s_ {0.50};

  // FreeSpace overtake-check window (matches Python).
  int    freespace_left_col_start_{0};
  int    freespace_left_col_end_  {3};
  double freespace_clear_thresh_  {0.3};
  // Lane-confidence threshold above which we consider the lane "dashed" /
  // overtakable. Python default 0.7.
  double lane_dashed_thresh_      {0.7};

  // CSV logging (Phase Final telemetry layer).
  bool        enable_csv_logging_{false};
  std::string csv_log_dir_{"/var/log/bfmc/planning"};
  std::ofstream csv_fsm_;
  std::uint64_t csv_tick_counter_{0};
  static constexpr std::uint64_t CSV_FLUSH_EVERY = 100;  // 100 ticks @20Hz = 5 s
  bool open_csv_logs_();
  void close_csv_logs_();
};

}  // namespace behavior_planner_cpp

#endif  // BEHAVIOR_PLANNER_CPP__BEHAVIOR_PLANNER_NODE_HPP_
