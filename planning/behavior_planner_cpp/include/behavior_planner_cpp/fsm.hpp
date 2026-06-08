#ifndef BEHAVIOR_PLANNER_CPP__FSM_HPP_
#define BEHAVIOR_PLANNER_CPP__FSM_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace behavior_planner_cpp
{

// ============================================================================
// Maneuver enum — mirrors Phase 5 spec, plus highway entry/exit per Phase Final
// ============================================================================
enum class Maneuver : std::uint8_t
{
  LANE_KEEP            = 0,
  STOP                 = 1,
  FOLLOW               = 2,    // tailing the vehicle ahead
  PEDESTRIAN_WAIT      = 3,
  CROSSWALK_SLOW       = 4,    // Phase Final: slow but DON'T stop at crosswalk
  ROUNDABOUT_ENTER     = 5,
  ROUNDABOUT_EXIT      = 6,
  PARK                 = 7,
  INTERSECTION_LEFT    = 8,
  INTERSECTION_RIGHT   = 9,
  HIGHWAY_ENTER        = 10,   // Phase Final: smooth-accel profile
  HIGHWAY_EXIT         = 11,   // Phase Final: smooth-decel profile
  EMERGENCY_STOP       = 12,
  // Race-day finals: lane-change overtake with explicit phasing.
  OVERTAKE             = 13,   // 3-phase: ramp → hold → return
};

const char * maneuver_to_str(Maneuver m);

// ============================================================================
// DetectionTimer — port of the Python BehaviorPlanner.DetectionTimer
// ============================================================================
// Hysteresis on per-class object detections so a single flickering frame
// cannot drive an FSM transition. Mirrors the old Python implementation:
//
//   • detected=true: arm first_seen_t_; after confirm_s consecutive seconds
//     of presence, flip to confirmed=true.
//   • detected=false: arm first_absent_t_; after clear_s consecutive seconds
//     of absence, flip to confirmed=false and reset closest_dist_.
//
// Time is supplied externally (ROS clock seconds) so the class has no ROS
// dependency and is unit-testable in isolation.
class DetectionTimer
{
public:
  explicit DetectionTimer(double confirm_s = 0.3, double clear_s = 0.8);

  void update(bool detected, double distance, double now);
  bool   confirmed()        const { return confirmed_; }
  double closest_distance() const { return closest_dist_; }
  void   reset();

private:
  double confirm_s_;
  double clear_s_;
  bool   confirmed_       {false};
  double first_seen_t_    {-1.0};
  double last_seen_t_     {-1.0};
  double first_absent_t_  {-1.0};
  double closest_dist_;
};

// ============================================================================
// Stopwatch — also from the Python planner
// ============================================================================
class Stopwatch
{
public:
  void   start(double now);
  void   reset();
  bool   running() const { return start_t_.has_value(); }
  double elapsed(double now) const;

private:
  std::optional<double> start_t_;
};

// ============================================================================
// FsmInputs — one tick's worth of trigger state, filled by the ROS node
// ============================================================================
struct FsmInputs
{
  // -- General signals --
  bool   state_valid{false};
  double speed{0.0};                 // m/s, from VehicleState.speed
  double imu_confidence{1.0};        // /fusion/imu_confidence
  double state_confidence{1.0};      // VehicleState.confidence

  // -- Priority triggers --
  bool   emergency{false};
  bool   pedestrian_ahead{false};
  double pedestrian_dist{1e9};
  bool   traffic_light_red_ahead{false};   // visual class OR V2X red state
  bool   stop_sign_ahead{false};
  double stop_sign_dist{1e9};
  bool   roadblock_ahead{false};
  bool   vehicle_ahead{false};
  double vehicle_dist{1e9};
  bool   parking_zone{false};        // from nav_flags or "parking" class
  bool   roundabout_zone{false};     // from nav_flags or "roundabout" class
  bool   roundabout_exiting{false};
  bool   crosswalk_ahead{false};     // Phase Final
  bool   highway_entering{false};    // Phase Final: from "highway_entry" class
  bool   highway_exiting{false};     // Phase Final: from "highway_exit" class
  bool   on_highway{false};          // Phase Final: from nav_flags (zones.yaml)

  // -- Overtake legality inputs (Phase Final) --
  bool   freespace_left_clear{false};
  bool   lane_dashed{false};         // lane_geometry.confidence > 0.7

  // -- Intersection turn hint (from path lookahead) --
  // 'L' / 'R' / unset (straight)
  std::optional<char> intersection_turn;

  // -- Lane validity --
  bool   lane_valid{true};

  // -- Race-day additions (BFMC sign set) --
  // no_entry  : "Do Not Enter" sign — treat as a hard STOP if observed in
  //             the forward path within `no_entry_dist_thresh`. Behavior
  //             is identical to roadblock but the cause is regulatory not
  //             physical.
  // priority  : Yellow diamond "you have right of way at the upcoming
  //             intersection". Modifier: in INTERSECTION_LEFT/RIGHT we
  //             allow a higher cruise speed (cap = `turn_speed_priority`)
  //             since we don't need to yield.
  // onewayroad: One-way arrow. Informational — current FSM never reverses,
  //             so no behavior change required; we record the flag for
  //             telemetry / future mission validation.
  bool   no_entry_ahead{false};
  double no_entry_dist{1e9};
  bool   has_priority{false};
  bool   on_oneway{false};
};

// ============================================================================
// FsmTuning — single source of truth for distance + speed thresholds
// ============================================================================
// Per Phase Final brief, all speeds live here. NO scattered constants in
// trigger code. ROS-overrideable via the node's declare_parameter calls.
struct FsmTuning
{
  // Confidence floors
  double low_imu_conf_thresh   {0.30};
  double low_state_conf_thresh {0.20};
  // Race-day soft-degradation. A transient dip below the conf floor does NOT
  // trigger EMERGENCY_STOP — the FSM instead emits LANE_KEEP @ degraded_speed
  // until the dip persists `degraded_persist_ticks` consecutive FSM ticks.
  // At 5 Hz default → 10 ticks = 2.0 s of sustained low-conf before hard stop.
  double degraded_speed        {0.20};   // m/s — slow-not-stop
  int    degraded_persist_ticks{10};

  // Distance gates (metres ahead, |y| < 1.5 m lateral)
  double pedestrian_dist_thresh    {0.80};
  double traffic_light_dist_thresh {1.20};
  double stop_sign_dist_thresh     {1.50};
  double roadblock_dist_thresh     {0.70};
  double vehicle_follow_dist_thresh{1.00};
  double crosswalk_dist_thresh     {2.00};
  double no_entry_dist_thresh      {1.50};   // race-day: BFMC "Do Not Enter"
  // Race-day fix: parking sign must be CLOSE before we commit to PARK.
  // Without this gate a single persistent false-positive parking sign
  // anywhere in the frame locked the FSM into PARK on the bench.
  double parking_dist_thresh       {0.80};
  double roundabout_dist_thresh    {1.50};

  // Speed table (m/s) — Phase Final
  double city_speed       {0.20};   // baseline LANE_KEEP cruise
  double highway_speed    {0.40};   // when on_highway
  double parking_speed    {0.08};
  double crosswalk_speed  {0.10};   // Phase Final: slow, NOT zero
  double follow_slow_speed{0.08};
  double tailing_min_speed{0.05};
  double turn_speed       {0.18};
  double roundabout_speed {0.15};
  // Race-day: with `priority` sign visible the intersection turn is
  // a right-of-way maneuver — we can hold a higher speed through it.
  double turn_speed_priority {0.25};

  // ---- Overtake (FOLLOW → lateral lane change → back to LANE_KEEP) -----
  // overtake_lateral_m : target left offset (≥ one lane width)
  // overtake_ramp_s    : sigmoid 0 → target
  // overtake_hold_s    : hold target (while passing leader)
  // overtake_return_s  : sigmoid target → 0 (return to right lane)
  // The TOTAL overtake duration is ramp+hold+return; on any abort signal
  // (lane no longer dashed, freespace no longer clear, new obstacle in
  // left lane) the FSM forces a fast return: skip to return phase from
  // wherever we are.
  double overtake_lateral_m  {0.40};
  double overtake_ramp_s     {1.0};
  double overtake_hold_s     {1.5};
  double overtake_return_s   {1.0};
  // Minimum speed before we'll attempt overtake (so we don't try at a stop)
  double overtake_min_speed  {0.15};
  // Overtake cruise speed — keep some momentum through the maneuver
  double overtake_speed      {0.30};

  // Stop-sign wait timer (seconds)
  double stop_sign_wait_s   {3.0};
  // Highway entry/exit smooth-accel ramp (seconds)
  double highway_ramp_s     {2.0};

  // Approach decel constants (Phase Final, sqrt(2·a·d))
  double a_max          {1.80};   // m/s² longitudinal deceleration cap
  double t_react        {0.15};   // s reaction time
  double stop_margin    {0.20};   // m safety margin past target
};

// ============================================================================
// Fsm — strict priority chain with hysteresis, stop-sign timer, highway, …
// ============================================================================
class Fsm
{
public:
  Fsm() = default;

  // Single tick. now_s is rclcpp Clock seconds (passed in so the class itself
  // is ROS-free). Returns the maneuver selected this tick.
  Maneuver step(const FsmInputs & in, const FsmTuning & t, double now_s);

  // Recommended cruise speed for the current maneuver, given the tuning
  // table. Speeds for STOP / WAIT / EMERGENCY are 0.
  double recommended_speed(const FsmTuning & t) const;

  Maneuver current()  const { return current_; }
  Maneuver previous() const { return previous_; }
  bool just_transitioned() const { return current_ != previous_; }

private:
  // Physics helper: stopping distance d_brake + d_react + d_margin.
  static double stopping_distance(double speed, const FsmTuning & t);
  // Physics helper: speed cap that lets us decel to v_min in `d` metres.
  static double approach_decel_speed(double speed, double d, const FsmTuning & t,
                                     double v_min);

  Maneuver current_{Maneuver::LANE_KEEP};
  Maneuver previous_{Maneuver::LANE_KEEP};

  // Wait-timer state (sub-FSM): STOP at stop sign for stop_sign_wait_s.
  Stopwatch stop_sign_wait_;
  // Highway entry ramp anchor (start time used to soften step inputs).
  Stopwatch highway_ramp_;

  // Last-tick recommended speed, set by step() so recommended_speed()
  // returns the same value the FSM internally computed (lets us reuse
  // approach_decel without recomputing in the caller).
  double last_recommended_speed_{0.0};

  // Race-day soft-degradation streak counter. Counts consecutive ticks
  // where the IMU OR state confidence is below the floor. Once it reaches
  // FsmTuning.degraded_persist_ticks the FSM commits to EMERGENCY_STOP;
  // before that, transient dips degrade to LANE_KEEP @ degraded_speed.
  int low_conf_streak_{0};

  // ---- Overtake sub-FSM state -----------------------------------------
  // OvertakePhase represents where in the 3-step maneuver we are. NONE
  // means no overtake in progress.
public:
  enum class OvertakePhase : std::uint8_t {
    NONE   = 0,   // not overtaking
    RAMP   = 1,   // sigmoid 0 → target
    HOLD   = 2,   // holding target (passing leader)
    RETURN = 3,   // sigmoid target → 0 (back to right lane)
  };
  OvertakePhase overtake_phase()  const { return overtake_phase_; }
  // Current commanded lateral offset (m, positive = left). Local planner
  // adds this to every TrajectoryPoint.y to execute the lane change.
  double lateral_offset()         const { return lateral_offset_; }
private:
  OvertakePhase overtake_phase_{OvertakePhase::NONE};
  Stopwatch     overtake_sw_;
  double        lateral_offset_{0.0};   // m, current commanded offset
};

}  // namespace behavior_planner_cpp

#endif  // BEHAVIOR_PLANNER_CPP__FSM_HPP_
