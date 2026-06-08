#include "behavior_planner_cpp/fsm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace behavior_planner_cpp
{

// ---- Maneuver names --------------------------------------------------------
const char * maneuver_to_str(Maneuver m)
{
  switch (m) {
    case Maneuver::LANE_KEEP:          return "LANE_KEEP";
    case Maneuver::STOP:               return "STOP";
    case Maneuver::FOLLOW:             return "FOLLOW";
    case Maneuver::PEDESTRIAN_WAIT:    return "PEDESTRIAN_WAIT";
    case Maneuver::CROSSWALK_SLOW:     return "CROSSWALK_SLOW";
    case Maneuver::ROUNDABOUT_ENTER:   return "ROUNDABOUT_ENTER";
    case Maneuver::ROUNDABOUT_EXIT:    return "ROUNDABOUT_EXIT";
    case Maneuver::PARK:               return "PARK";
    case Maneuver::INTERSECTION_LEFT:  return "INTERSECTION_LEFT";
    case Maneuver::INTERSECTION_RIGHT: return "INTERSECTION_RIGHT";
    case Maneuver::HIGHWAY_ENTER:      return "HIGHWAY_ENTER";
    case Maneuver::HIGHWAY_EXIT:       return "HIGHWAY_EXIT";
    case Maneuver::EMERGENCY_STOP:     return "EMERGENCY_STOP";
    case Maneuver::OVERTAKE:           return "OVERTAKE";
  }
  return "UNKNOWN";
}

// ---- DetectionTimer --------------------------------------------------------
DetectionTimer::DetectionTimer(double confirm_s, double clear_s)
: confirm_s_(confirm_s),
  clear_s_(clear_s),
  closest_dist_(std::numeric_limits<double>::infinity())
{}

void DetectionTimer::update(bool detected, double distance, double now)
{
  if (detected) {
    first_absent_t_ = -1.0;
    last_seen_t_    = now;
    if (first_seen_t_ < 0.0) first_seen_t_ = now;
    if (!confirmed_ && (now - first_seen_t_) >= confirm_s_) confirmed_ = true;
    if (confirmed_) closest_dist_ = std::min(closest_dist_, distance);
  } else {
    first_seen_t_ = -1.0;
    if (first_absent_t_ < 0.0) first_absent_t_ = now;
    if (confirmed_ && (now - first_absent_t_) >= clear_s_) {
      confirmed_    = false;
      closest_dist_ = std::numeric_limits<double>::infinity();
    }
  }
}

void DetectionTimer::reset()
{
  confirmed_       = false;
  first_seen_t_    = -1.0;
  last_seen_t_     = -1.0;
  first_absent_t_  = -1.0;
  closest_dist_    = std::numeric_limits<double>::infinity();
}

// ---- Stopwatch -------------------------------------------------------------
void Stopwatch::start(double now)
{
  if (!start_t_.has_value()) start_t_ = now;
}
void Stopwatch::reset() { start_t_.reset(); }
double Stopwatch::elapsed(double now) const
{
  return start_t_.has_value() ? (now - *start_t_) : 0.0;
}

// ---- Physics helpers -------------------------------------------------------
double Fsm::stopping_distance(double speed, const FsmTuning & t)
{
  const double v = std::abs(speed);
  const double d_brake = (t.a_max > 1e-6) ? (v * v) / (2.0 * t.a_max) : 0.0;
  const double d_react = v * t.t_react;
  return d_brake + d_react + t.stop_margin;
}

double Fsm::approach_decel_speed(double speed, double d, const FsmTuning & t,
                                 double v_min)
{
  if (d <= 0.0) return v_min;
  // v_allow such that v_allow^2 / (2·a) = d, i.e. v = sqrt(2·a·d).
  const double v_allow = std::sqrt(2.0 * t.a_max * d);
  return std::max(v_min, std::min(v_allow, std::abs(speed)));
}

// ---- Fsm::step -------------------------------------------------------------
//
// Strict-priority chain, hysteresis applied UPSTREAM by the node (DetectionTimer
// wraps each class in build_inputs_ before this is called). This function is the
// pure mapping (FsmInputs, FsmTuning) → Maneuver + recommended speed.
//
// Priority order (highest → lowest):
//   1. !state_valid                              ⇒ EMERGENCY_STOP (state loss)
//   2. emergency || imu_conf low || state_conf low ⇒ EMERGENCY_STOP
//   3. pedestrian within physics-stop distance   ⇒ PEDESTRIAN_WAIT (v=0)
//   4. (overtake / tail) vehicle ahead within physics-stop distance:
//        4a. lane dashed + freespace_left_clear  ⇒ FOLLOW (with intent to
//                                                          overtake — local
//                                                          planner handles
//                                                          the geometry)
//        4b. else                                ⇒ FOLLOW (tail-decel speed)
//   5. roadblock close                            ⇒ STOP
//   6. red light close                            ⇒ STOP
//   7. stop sign close                            ⇒ STOP (handled with a
//                                                          stop_sign_wait_
//                                                          timer below)
//   8. crosswalk close                            ⇒ CROSSWALK_SLOW
//   9. parking zone                               ⇒ PARK
//  10. roundabout zone                            ⇒ ROUNDABOUT_ENTER/EXIT
//  11. highway entering                           ⇒ HIGHWAY_ENTER
//  12. highway exiting                            ⇒ HIGHWAY_EXIT
//  13. intersection turn from path lookahead     ⇒ INTERSECTION_LEFT/RIGHT
//  14. default                                    ⇒ LANE_KEEP
//
Maneuver Fsm::step(const FsmInputs & in, const FsmTuning & t, double now_s)
{
  previous_ = current_;

  // ---- Hard EMERGENCY stops (1, 2) ----------------------------------------
  // !state_valid (stale /vehicle/state) and explicit /safety/emergency are
  // ALWAYS hard stops — these are upstream gates that have their own
  // hysteresis (emergency_stop_node persistence, system_state_machine
  // ready hysteresis).
  //
  // Any of these aborts an in-progress OVERTAKE — clear the sub-FSM so we
  // don't resume mid-maneuver when the emergency clears. The lateral
  // offset is forced back to 0 immediately (sharp return) since this is
  // an emergency, not a graceful abort.
  auto abort_overtake_ = [this]() {
    overtake_phase_ = OvertakePhase::NONE;
    overtake_sw_.reset();
    lateral_offset_ = 0.0;
  };
  if (!in.state_valid) {
    current_ = Maneuver::EMERGENCY_STOP;
    last_recommended_speed_ = 0.0;
    low_conf_streak_ = 0;
    abort_overtake_();
    return current_;
  }
  if (in.emergency) {
    current_ = Maneuver::EMERGENCY_STOP;
    last_recommended_speed_ = 0.0;
    low_conf_streak_ = 0;
    abort_overtake_();
    return current_;
  }
  // Confidence floor: SOFT-degrade first, then HARD-stop if it persists.
  // A transient single-tick dip emits LANE_KEEP @ degraded_speed (default
  // 0.20 m/s) rather than slamming to zero, per race-day brief.
  const bool low_conf =
      (in.imu_confidence   < t.low_imu_conf_thresh) ||
      (in.state_confidence < t.low_state_conf_thresh);
  if (low_conf) {
    low_conf_streak_ = std::min(low_conf_streak_ + 1,
                                t.degraded_persist_ticks + 1);
    if (low_conf_streak_ >= t.degraded_persist_ticks) {
      current_ = Maneuver::EMERGENCY_STOP;
      last_recommended_speed_ = 0.0;
      abort_overtake_();
      return current_;
    }
    // Transient: crawl at degraded_speed, keep the lane.
    // Also abort any in-progress overtake — low confidence in our state
    // means lane geometry is unreliable, can't trust the offset.
    current_ = Maneuver::LANE_KEEP;
    last_recommended_speed_ = t.degraded_speed;
    abort_overtake_();
    return current_;
  }
  low_conf_streak_ = 0;

  // ---- Pedestrian (3) -----------------------------------------------------
  // Use physics-stop distance so a fast vehicle reacts earlier than the raw
  // class threshold would suggest. matches the Python implementation.
  // Pedestrian danger aborts any active overtake — we go straight to STOP.
  if (in.pedestrian_ahead) {
    const double d_danger = std::max(
        t.pedestrian_dist_thresh, stopping_distance(in.speed, t));
    if (in.pedestrian_dist <= d_danger) {
      current_ = Maneuver::PEDESTRIAN_WAIT;
      last_recommended_speed_ = 0.0;
      abort_overtake_();
      return current_;
    }
  }

  // ---- Vehicle ahead (4) — full overtake sub-FSM --------------------------
  //
  // If we entered OVERTAKE on the previous tick, finish the phase chain
  // unconditionally — but with continuous safety re-checks. If anything
  // bad happens mid-maneuver (lane no longer dashed, freespace no longer
  // clear, lost state validity), we fast-forward to the RETURN phase to
  // bring the car back into the right lane.
  const auto sigmoid_ = [](double u) -> double {
    // smoothstep: 0 at u≤0, 1 at u≥1, smooth-S in between.
    if (u <= 0.0) return 0.0;
    if (u >= 1.0) return 1.0;
    return u * u * (3.0 - 2.0 * u);
  };

  if (overtake_phase_ != OvertakePhase::NONE) {
    const double t_elapsed = overtake_sw_.elapsed(now_s);
    const double t_ramp    = t.overtake_ramp_s;
    const double t_hold    = t_ramp + t.overtake_hold_s;
    const double t_total   = t_hold + t.overtake_return_s;
    const bool   abort     = !in.lane_dashed || !in.freespace_left_clear;

    if (abort && overtake_phase_ != OvertakePhase::RETURN) {
      // Forced fast return — jump the stopwatch to the start of RETURN.
      overtake_sw_.reset();
      overtake_sw_.start(now_s);
      // Compute pseudo-elapsed so we begin returning from the CURRENT offset
      // rather than 0.
      overtake_phase_ = OvertakePhase::RETURN;
    }

    if (overtake_phase_ == OvertakePhase::RAMP) {
      lateral_offset_ = t.overtake_lateral_m * sigmoid_(t_elapsed / t_ramp);
      if (t_elapsed >= t_ramp) overtake_phase_ = OvertakePhase::HOLD;
    } else if (overtake_phase_ == OvertakePhase::HOLD) {
      lateral_offset_ = t.overtake_lateral_m;
      if (t_elapsed >= t_hold) overtake_phase_ = OvertakePhase::RETURN;
    } else if (overtake_phase_ == OvertakePhase::RETURN) {
      const double u = (t_elapsed - t_hold) / t.overtake_return_s;
      lateral_offset_ = t.overtake_lateral_m * (1.0 - sigmoid_(u));
      if (t_elapsed >= t_total) {
        // Maneuver complete — back to LANE_KEEP.
        overtake_phase_ = OvertakePhase::NONE;
        overtake_sw_.reset();
        lateral_offset_ = 0.0;
      }
    }

    if (overtake_phase_ != OvertakePhase::NONE) {
      current_ = Maneuver::OVERTAKE;
      last_recommended_speed_ = t.overtake_speed;
      return current_;
    }
    // fell through — phase chain finished, continue with normal eval.
  } else {
    // Not currently overtaking — decay any residual offset to 0.
    lateral_offset_ = 0.0;
  }

  // Re-evaluate vehicle-ahead.
  if (in.vehicle_ahead) {
    const double d_engage = std::max(
        t.vehicle_follow_dist_thresh, stopping_distance(in.speed, t));
    if (in.vehicle_dist <= d_engage) {
      // Initiate OVERTAKE iff all preconditions met:
      //   - dashed lane (legal lane change)
      //   - left lane clear (no oncoming / free-space confirmed)
      //   - we're moving fast enough that the maneuver makes sense
      //   - not currently engaging another mandatory state (already gated
      //     by priority order — we'd have returned earlier)
      if (in.lane_dashed && in.freespace_left_clear &&
          in.speed >= t.overtake_min_speed)
      {
        overtake_phase_ = OvertakePhase::RAMP;
        overtake_sw_.start(now_s);
        lateral_offset_ = 0.0;  // begins at 0, ramps next tick
        current_ = Maneuver::OVERTAKE;
        last_recommended_speed_ = t.overtake_speed;
        return current_;
      }
      // Either lane is solid, left lane not clear, or we're already slow.
      // Fall back to tailing: ease down toward stop_margin behind the leader.
      const double v_tail = approach_decel_speed(
          in.speed, in.vehicle_dist - t.stop_margin, t, t.tailing_min_speed);
      current_ = Maneuver::FOLLOW;
      last_recommended_speed_ = v_tail;
      return current_;
    }
  }

  // ---- Roadblock (5) ------------------------------------------------------
  if (in.roadblock_ahead) {
    current_ = Maneuver::STOP;
    last_recommended_speed_ = 0.0;
    return current_;
  }

  // ---- No-entry sign (5b) -------------------------------------------------
  // BFMC "Do Not Enter" — regulatory hard stop. Same priority as roadblock:
  // if the sign is observed in the forward path within the threshold, we
  // STOP and let the mission supervisor / operator reroute. We do NOT use
  // the stop-sign 3-s wait timer here: no_entry is a permanent prohibition,
  // not a yield-and-go.
  if (in.no_entry_ahead && in.no_entry_dist <= t.no_entry_dist_thresh) {
    current_ = Maneuver::STOP;
    last_recommended_speed_ = 0.0;
    return current_;
  }

  // ---- Red traffic light (6) ----------------------------------------------
  // Source is either visual "trafficlight" class (default behaviour: treat
  // as red) OR V2X-confirmed red state from /v2x/semaphores via the node.
  if (in.traffic_light_red_ahead) {
    current_ = Maneuver::STOP;
    last_recommended_speed_ = 0.0;
    stop_sign_wait_.reset();   // not a stop-sign wait — clear any leftover
    return current_;
  }

  // ---- Stop sign (7) ------------------------------------------------------
  // Sub-FSM: STOP at sign, hold v=0 for stop_sign_wait_s, then release.
  // The "release" exits the if-branch and falls through to default LANE_KEEP
  // on the next tick once the sign is past us (DetectionTimer's clear_s will
  // drop stop_sign_ahead within ~0.6 s).
  if (in.stop_sign_ahead) {
    // Approach phase: ramp speed down using physics decel.
    const double v_app = approach_decel_speed(
        in.speed, in.stop_sign_dist - t.stop_margin, t, 0.0);
    if (std::abs(in.speed) > 0.05) {
      // Still moving — STOP maneuver, target speed = decel curve.
      current_ = Maneuver::STOP;
      last_recommended_speed_ = v_app;
      return current_;
    }
    // We've stopped. Hold for stop_sign_wait_s.
    stop_sign_wait_.start(now_s);
    if (stop_sign_wait_.elapsed(now_s) < t.stop_sign_wait_s) {
      current_ = Maneuver::STOP;
      last_recommended_speed_ = 0.0;
      return current_;
    }
    // Wait satisfied — fall through to lower-priority eval.
    stop_sign_wait_.reset();
  }

  // ---- Crosswalk (8) — slow, NOT stop -------------------------------------
  if (in.crosswalk_ahead) {
    current_ = Maneuver::CROSSWALK_SLOW;
    last_recommended_speed_ = t.crosswalk_speed;
    return current_;
  }

  // ---- Parking (9) --------------------------------------------------------
  if (in.parking_zone) {
    current_ = Maneuver::PARK;
    last_recommended_speed_ = t.parking_speed;
    return current_;
  }

  // ---- Roundabout (10) ----------------------------------------------------
  if (in.roundabout_zone) {
    current_ = in.roundabout_exiting
      ? Maneuver::ROUNDABOUT_EXIT
      : Maneuver::ROUNDABOUT_ENTER;
    last_recommended_speed_ = t.roundabout_speed;
    return current_;
  }

  // ---- Highway entry / exit (11, 12) --------------------------------------
  // Smooth-accel / smooth-decel via a linear ramp anchored on first frame
  // the trigger fires. local_path_planner / NMPC will further smooth.
  if (in.highway_entering) {
    highway_ramp_.start(now_s);
    const double a = std::clamp(highway_ramp_.elapsed(now_s) / t.highway_ramp_s,
                                0.0, 1.0);
    current_ = Maneuver::HIGHWAY_ENTER;
    last_recommended_speed_ = t.city_speed + a * (t.highway_speed - t.city_speed);
    return current_;
  }
  if (in.highway_exiting) {
    highway_ramp_.start(now_s);
    const double a = std::clamp(highway_ramp_.elapsed(now_s) / t.highway_ramp_s,
                                0.0, 1.0);
    current_ = Maneuver::HIGHWAY_EXIT;
    last_recommended_speed_ = t.highway_speed - a * (t.highway_speed - t.city_speed);
    return current_;
  }
  // No active highway-transition → drop the ramp anchor.
  highway_ramp_.reset();

  // ---- Intersection turn hint (13) ---------------------------------------
  if (in.intersection_turn.has_value()) {
    current_ = (*in.intersection_turn == 'L')
      ? Maneuver::INTERSECTION_LEFT
      : Maneuver::INTERSECTION_RIGHT;
    // Race-day: "priority" sign visible = right-of-way at this intersection.
    // We can hold a higher cruise (turn_speed_priority) instead of the
    // conservative default turn_speed. Falls back to turn_speed if the
    // priority detector hasn't confirmed it.
    last_recommended_speed_ = in.has_priority ? t.turn_speed_priority
                                              : t.turn_speed;
    return current_;
  }

  // ---- Default LANE_KEEP (14) --------------------------------------------
  // On highway sections we cruise faster even without entry/exit triggers.
  current_ = Maneuver::LANE_KEEP;
  last_recommended_speed_ = in.on_highway ? t.highway_speed : t.city_speed;
  return current_;
}

double Fsm::recommended_speed(const FsmTuning & /*t*/) const
{
  // step() already populated last_recommended_speed_ with the correct value
  // (incl. physics-decel for STOP/FOLLOW). Caller just reads it.
  return last_recommended_speed_;
}

}  // namespace behavior_planner_cpp
