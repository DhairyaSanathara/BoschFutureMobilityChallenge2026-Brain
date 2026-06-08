#include "local_path_planner_cpp/local_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace local_path_planner_cpp
{

LocalPathPlannerNode::LocalPathPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_path_planner_node", options)
{
  // ---- Parameters --------------------------------------------------------
  planner_hz_          = declare_parameter<double>("planner_hz",          planner_hz_);
  lookahead_m_         = declare_parameter<double>("lookahead_m",         lookahead_m_);
  num_points_          = declare_parameter<int>("num_points",             num_points_);
  a_max_               = declare_parameter<double>("a_max",               a_max_);
  a_lat_max_           = declare_parameter<double>("a_lat_max",           a_lat_max_);
  min_curve_speed_     = declare_parameter<double>("min_curve_speed",     min_curve_speed_);
  overtake_lateral_m_  = declare_parameter<double>("overtake_lateral_m",  overtake_lateral_m_);
  overtake_distance_m_ = declare_parameter<double>("overtake_distance_m", overtake_distance_m_);
  obstacle_radius_     = declare_parameter<double>("obstacle_radius",     obstacle_radius_);
  obstacle_lookahead_max_ = declare_parameter<double>("obstacle_lookahead_max", obstacle_lookahead_max_);
  fallback_poly_a_     = declare_parameter<double>("fallback_poly_a",     fallback_poly_a_);
  fallback_poly_b_     = declare_parameter<double>("fallback_poly_b",     fallback_poly_b_);
  fallback_poly_c_     = declare_parameter<double>("fallback_poly_c",     fallback_poly_c_);
  lane_min_confidence_ = declare_parameter<double>("lane_min_confidence", lane_min_confidence_);
  stale_lane_s_        = declare_parameter<double>("stale_lane_s",        stale_lane_s_);
  stale_state_s_       = declare_parameter<double>("stale_state_s",       stale_state_s_);
  stale_objects_s_     = declare_parameter<double>("stale_objects_s",     stale_objects_s_);

  // ---- ROS interfaces ----------------------------------------------------
  auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto be  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_state_    = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", rel,
    std::bind(&LocalPathPlannerNode::on_state, this, std::placeholders::_1));
  sub_lane_     = create_subscription<bfmc_msgs::msg::LaneGeometry>(
    "/perception/lane_geometry", be,
    std::bind(&LocalPathPlannerNode::on_lane, this, std::placeholders::_1));
  sub_behavior_ = create_subscription<std_msgs::msg::String>(
    "/planning/behavior_state", rel,
    std::bind(&LocalPathPlannerNode::on_behavior, this, std::placeholders::_1));
  sub_target_v_ = create_subscription<std_msgs::msg::Float32>(
    "/planning/target_speed", rel,
    std::bind(&LocalPathPlannerNode::on_target_v, this, std::placeholders::_1));
  sub_objects_  = create_subscription<bfmc_msgs::msg::ObjectArray>(
    "/perception/tracked_objects", rel,
    std::bind(&LocalPathPlannerNode::on_objects, this, std::placeholders::_1));

  // Race-day overtake: lateral offset from behavior_planner.
  sub_lat_offset_ = create_subscription<std_msgs::msg::Float32>(
    "/planning/lateral_offset", rel,
    [this](std_msgs::msg::Float32::SharedPtr m) {
      if (std::isfinite(m->data)) {
        // Clamp defensively to ±0.6 m — anything beyond is implausible at
        // BFMC scale (lane width ~0.4 m).
        const double clamped = std::clamp(static_cast<double>(m->data), -0.6, 0.6);
        lateral_offset_cmd_.store(clamped, std::memory_order_relaxed);
      }
    });

  pub_path_      = create_publisher<std_msgs::msg::String>("/planning/local_path", rel);
  pub_path_conf_ = create_publisher<std_msgs::msg::Float64>("/planning/path_confidence", be);

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / planner_hz_)),
    std::bind(&LocalPathPlannerNode::tick, this));

  RCLCPP_INFO(get_logger(),
    "local_path_planner_cpp: %.1f Hz, L=%.2fm, N=%d, a_lat=%.2f, "
    "obstacle_radius=%.2fm",
    planner_hz_, lookahead_m_, num_points_, a_lat_max_, obstacle_radius_);
}

/* =========================== Callbacks ============================ */

void LocalPathPlannerNode::on_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_state_    = *msg;
  have_state_      = true;
  last_state_time_ = this->now();
}
void LocalPathPlannerNode::on_lane(const bfmc_msgs::msg::LaneGeometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_lane_    = *msg;
  have_lane_      = true;
  last_lane_time_ = this->now();
}
void LocalPathPlannerNode::on_behavior(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  behavior_state_ = msg->data;
}
void LocalPathPlannerNode::on_target_v(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  target_speed_ = static_cast<double>(msg->data);
}
void LocalPathPlannerNode::on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_objects_    = *msg;
  have_objects_      = true;
  last_objects_time_ = this->now();
}

/* =========================== Tick ============================ */

std::vector<double> LocalPathPlannerNode::make_stations(double L, int N) const
{
  std::vector<double> s;
  s.reserve(N);
  const double step = (N > 1) ? L / static_cast<double>(N - 1) : 0.0;
  for (int i = 0; i < N; ++i) s.push_back(i * step);
  return s;
}

std::vector<TrajectoryPoint> LocalPathPlannerNode::generate_(
    const std::string & behavior,
    const Poly3 & lane,
    const std::vector<double> & stations,
    double speed,
    double target_speed)
{
  std::vector<TrajectoryPoint> pts;
  pts.reserve(stations.size());

  // Behavior dispatch — mirrors the Python state-machine 1:1.
  if (behavior == "FOLLOW") {
    const auto vprof = velocity_decel_to_target(
        speed, target_speed, lookahead_m_, stations, a_max_);
    for (std::size_t i = 0; i < stations.size(); ++i) {
      const double x = stations[i];
      pts.push_back({x, eval_quadratic(lane, x), heading_quadratic(lane, x),
                     curvature_quadratic(lane, x), vprof[i]});
    }
  }
  else if (behavior == "STOP") {
    const auto vprof = velocity_decel_to_stop(
        speed, lookahead_m_, stations, a_max_);
    for (std::size_t i = 0; i < stations.size(); ++i) {
      const double x = stations[i];
      pts.push_back({x, eval_quadratic(lane, x), heading_quadratic(lane, x),
                     curvature_quadratic(lane, x), vprof[i]});
    }
  }
  else if (behavior == "PEDESTRIAN_WAIT" || behavior == "EMERGENCY_STOP") {
    // Hold-and-zero: keep heading, zero velocity.
    for (double x : stations) {
      pts.push_back({x, eval_quadratic(lane, x), heading_quadratic(lane, x),
                     curvature_quadratic(lane, x), 0.0});
    }
  }
  else {
    // LANE_KEEP, INTERSECTION_*, ROUNDABOUT_*, PARK, CROSSWALK_SLOW,
    // HIGHWAY_*, OVERTAKE  → lane-follow at target_speed. The FSM already
    // set target_speed to the maneuver-appropriate cruise value. The
    // OVERTAKE lateral shift is applied separately in tick() via the
    // /planning/lateral_offset stream.
    for (double x : stations) {
      pts.push_back({x, eval_quadratic(lane, x), heading_quadratic(lane, x),
                     curvature_quadratic(lane, x), target_speed});
    }
  }
  return pts;
}

void LocalPathPlannerNode::apply_curvature_speed_limit_(
    std::vector<TrajectoryPoint> & pts) const
{
  // From Bosch autonomous-racing literature: v ≤ sqrt(a_lat / |κ|).
  // Mirrors the Python implementation; clamped to min_curve_speed_ so we
  // never command zero on a curve while otherwise free to drive.
  for (auto & p : pts) {
    if (p.target_speed <= 0.0) continue;
    const double k = std::abs(p.curvature);
    if (k > 1e-3) {
      const double v_curve = std::sqrt(a_lat_max_ / k);
      p.target_speed = std::max(min_curve_speed_,
                                std::min(p.target_speed, v_curve));
    }
  }
}

bool LocalPathPlannerNode::apply_obstacle_clip_(
    std::vector<TrajectoryPoint> & pts)
{
  // Phase B: scan tracked objects for collisions with the planned path.
  // We treat each obstacle as a disk of radius obstacle_radius_; if any path
  // point is inside the disk AND inside the obstacle_lookahead_max_ forward
  // window, we mark THAT point's forward-station as the brake-by point and
  // override the velocity profile to decel-to-stop at that distance.
  //
  // Returns true if a clip was applied so the caller can lower path_confidence.
  std::lock_guard<std::mutex> lk(cache_mtx_);
  if (!have_objects_) return false;
  if ((this->now() - last_objects_time_).seconds() > stale_objects_s_) return false;

  double brake_at = std::numeric_limits<double>::infinity();
  for (const auto & o : latest_objects_.objects) {
    if (o.position.x <= 0.0 || o.position.x > obstacle_lookahead_max_) continue;
    for (const auto & p : pts) {
      const double dx = o.position.x - p.x;
      const double dy = o.position.y - p.y;
      if (dx * dx + dy * dy < obstacle_radius_ * obstacle_radius_) {
        brake_at = std::min(brake_at, o.position.x);
        break;       // this obstacle is on the path — next obstacle
      }
    }
  }
  if (!std::isfinite(brake_at)) return false;

  // Decel-to-zero, anchored on brake_at so we stop at the obstacle.
  for (auto & p : pts) {
    if (p.x >= brake_at) { p.target_speed = 0.0; continue; }
    // sqrt-decel from current v0 to 0 by brake_at, evaluated at p.x.
    const double v0 = have_state_ ? std::abs(latest_state_.speed) : p.target_speed;
    const double a_needed = (v0 > 1e-3 && brake_at > 1e-3) ?
                            (v0 * v0) / (2.0 * brake_at) : a_max_;
    const double a = std::min(a_needed, a_max_);
    const double v2 = std::max(0.0, v0 * v0 - 2.0 * a * p.x);
    p.target_speed = std::min(p.target_speed, std::sqrt(v2));
  }
  return true;
}

std::string LocalPathPlannerNode::to_json_(
    const std::string & behavior,
    const std::vector<TrajectoryPoint> & pts) const
{
  // Same schema as the Python local_path_planner so trajectory_generator
  // doesn't need to change.
  std::ostringstream os;
  os << std::fixed;
  os << "{\"frame\":\"vehicle\",\"behavior\":\"" << behavior << "\","
     << "\"num_points\":" << pts.size() << ",\"points\":[";
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const auto & p = pts[i];
    os << std::setprecision(4)
       << "{\"x\":" << p.x
       << ",\"y\":" << p.y
       << ",\"heading\":" << p.heading
       << ",\"curvature\":" << p.curvature
       << ",\"target_speed\":" << p.target_speed << "}";
    if (i + 1 < pts.size()) os << ",";
  }
  os << "]}";
  return os.str();
}

void LocalPathPlannerNode::tick()
{
  // Snapshot inputs.
  Poly3   lane{};
  double  lane_conf{0.0};
  double  speed{0.0};
  double  tspeed{0.0};
  std::string behavior;
  bool    have_fresh_lane = false;
  {
    std::lock_guard<std::mutex> lk(cache_mtx_);
    const auto t_now = this->now();
    const bool state_fresh = have_state_ &&
      (t_now - last_state_time_).seconds() < stale_state_s_;
    const bool lane_fresh  = have_lane_ &&
      (t_now - last_lane_time_).seconds() < stale_lane_s_;
    speed     = state_fresh ? latest_state_.speed : 0.0;
    tspeed    = target_speed_;
    behavior  = behavior_state_;

    if (lane_fresh && latest_lane_.center_poly.size() >= 3 &&
        latest_lane_.confidence >= lane_min_confidence_)
    {
      lane = { latest_lane_.center_poly[0],
               latest_lane_.center_poly[1],
               latest_lane_.center_poly[2] };
      lane_conf = latest_lane_.confidence;
      have_fresh_lane = true;
    } else {
      // Emergency fallback path: straight ahead via fallback_poly_*.
      lane = { fallback_poly_a_, fallback_poly_b_, fallback_poly_c_ };
      lane_conf = 0.0;
    }
  }

  if (behavior.empty()) behavior = "LANE_KEEP";

  // Generate, then constrain, then collision-check.
  const auto stations = make_stations(lookahead_m_, num_points_);
  auto pts = generate_(behavior, lane, stations, speed, tspeed);

  // Race-day overtake: apply commanded lateral offset (m, +ve = left)
  // BEFORE obstacle clip — so the obstacle check evaluates the SHIFTED
  // path, allowing the car to pass a leader that sits on the centerline.
  // Heading is recomputed by finite differences so the trajectory is
  // tangent to the new shifted path (a constant y-bias keeps headings
  // unchanged, but during ramp/return the y values are time-varying
  // along x → heading deflects slightly toward the lane change).
  const double lat_cmd = lateral_offset_cmd_.load(std::memory_order_relaxed);
  if (std::abs(lat_cmd) > 1e-3) {
    for (auto & p : pts) p.y += lat_cmd;
    // Recompute heading from finite differences on the SHIFTED path so
    // the controller doesn't chase a discontinuous y while keeping the
    // old heading. We do this only when an overtake is active to avoid
    // wasted work on the lane-keep happy path.
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      const double dx = pts[i + 1].x - pts[i].x;
      const double dy = pts[i + 1].y - pts[i].y;
      if (std::abs(dx) > 1e-6) pts[i].heading = std::atan2(dy, dx);
    }
  }

  apply_curvature_speed_limit_(pts);
  const bool obstacle_clipped = apply_obstacle_clip_(pts);

  // Path confidence: lane confidence × (1 - 0.5 * obstacle_clipped).
  // Floor of 0.0; 1.0 when lane is clean and nothing on path.
  double path_conf = std::clamp(lane_conf, 0.0, 1.0);
  if (obstacle_clipped) path_conf *= 0.5;
  if (!have_fresh_lane) path_conf = 0.0;

  // Publish.
  std_msgs::msg::String pmsg; pmsg.data = to_json_(behavior, pts);
  pub_path_->publish(pmsg);
  std_msgs::msg::Float64 cmsg; cmsg.data = path_conf;
  pub_path_conf_->publish(cmsg);
}

}  // namespace local_path_planner_cpp
