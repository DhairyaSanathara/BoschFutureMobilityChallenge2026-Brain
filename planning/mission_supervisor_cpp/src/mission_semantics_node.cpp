#include "mission_supervisor_cpp/mission_semantics_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mission_supervisor_cpp
{

namespace {

// Helper — try several keys (some YAMLs use string ids, some int ids).
std::string yaml_node_id(const YAML::Node & n) {
  if (n.IsScalar()) return n.as<std::string>();
  return {};
}

}  // namespace

MissionSemanticsNode::MissionSemanticsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_semantics_node", options)
{
  supervisor_hz_   = declare_parameter<double>     ("supervisor_hz",   supervisor_hz_);
  graphml_path_    = declare_parameter<std::string>(
      "graphml_path",
      ament_index_cpp::get_package_share_directory("global_path_planner_cpp")
          + "/maps/Competition_track_graph.graphml");
  config_dir_      = declare_parameter<std::string>(
      "config_dir",
      ament_index_cpp::get_package_share_directory("mission_supervisor_cpp")
          + "/config");
  mission_yaml_    = declare_parameter<std::string>("mission_yaml",
                       config_dir_ + "/competition_mission.yaml");
  semantics_yaml_  = declare_parameter<std::string>("semantics_yaml",
                       config_dir_ + "/competition_semantics.yaml");
  speed_yaml_      = declare_parameter<std::string>("speed_yaml",
                       config_dir_ + "/competition_speed_profiles.yaml");
  risk_yaml_       = declare_parameter<std::string>("risk_yaml",
                       config_dir_ + "/competition_risk_zones.yaml");
  overrides_yaml_  = declare_parameter<std::string>("overrides_yaml",
                       config_dir_ + "/competition_behavior_overrides.yaml");
  hys_distance_margin_m_ = declare_parameter<double>(
      "hys_distance_margin_m", hys_distance_margin_m_);
  hys_persist_s_         = declare_parameter<double>(
      "hys_persist_s",         hys_persist_s_);

  // ---- Load graph ---------------------------------------------------------
  std::string err;
  if (!graph_.load(graphml_path_, err)) {
    RCLCPP_ERROR(get_logger(), "MISSION: graph load failed: %s", err.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "MISSION: graph loaded — %zu nodes, %zu edges",
                graph_.n_nodes(), graph_.n_edges());
  }

  // ---- Load YAMLs ---------------------------------------------------------
  bool all_ok = true;
  all_ok &= load_mission_(mission_yaml_);
  all_ok &= load_semantics_(semantics_yaml_);
  all_ok &= load_speed_profiles_(speed_yaml_);
  all_ok &= load_risk_zones_(risk_yaml_);
  all_ok &= load_behavior_overrides_(overrides_yaml_);
  if (!all_ok) {
    RCLCPP_WARN(get_logger(),
        "MISSION: some YAMLs failed to load — degraded mode "
        "(see earlier errors). Race-day decision: KEEP RUNNING.");
  }

  // ---- Validate mission against graph -------------------------------------
  validate_mission_();

  // ---- Build the mission Path + RViz markers once -------------------------
  build_competition_path_();
  build_marker_array_();

  // ---- ROS interface ------------------------------------------------------
  auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto be  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  // /planning/global_path: transient_local so late lifecycle nodes (behavior /
  // controller, activated T+2 s) see the last sample without us having to
  // republish on subscription. The 1 Hz timer below still ticks for live
  // updates.
  auto rel_latched = rclcpp::QoS(rclcpp::KeepLast(2)).reliable()
                        .durability(rclcpp::DurabilityPolicy::TransientLocal);

  sub_state_ = create_subscription<bfmc_msgs::msg::VehicleState>(
      "/vehicle/state", rel,
      std::bind(&MissionSemanticsNode::on_state_, this, std::placeholders::_1));
  sub_imu_conf_ = create_subscription<std_msgs::msg::Float64>(
      "/fusion/imu_confidence", be,
      std::bind(&MissionSemanticsNode::on_imu_conf_, this, std::placeholders::_1));

  pub_global_path_      = create_publisher<nav_msgs::msg::Path>(
      "/planning/global_path",      rel_latched);
  pub_competition_path_ = create_publisher<nav_msgs::msg::Path>(
      "/planning/competition_path", rel_latched);
  pub_markers_          = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/competition_markers", rel);
  pub_nav_flag_         = create_publisher<std_msgs::msg::String>(
      "/navigation/flags",          rel);
  pub_current_node_     = create_publisher<std_msgs::msg::Int32>(
      "/planning/current_node",     rel);
  pub_current_region_   = create_publisher<std_msgs::msg::String>(
      "/planning/current_region",   rel);
  pub_current_behavior_ = create_publisher<std_msgs::msg::String>(
      "/planning/current_behavior", rel);
  pub_current_speed_    = create_publisher<std_msgs::msg::Float64>(
      "/planning/current_speed_limit", rel);
  pub_progress_         = create_publisher<std_msgs::msg::Float64>(
      "/planning/mission_progress", rel);
  pub_score_est_        = create_publisher<std_msgs::msg::Float64>(
      "/planning/mission_score_estimate", be);
  pub_loc_conf_         = create_publisher<std_msgs::msg::Float64>(
      "/planning/localization_confidence", rel);

  // First-shot publication — pre-arm before the rest of the stack activates.
  pub_global_path_->publish(competition_path_msg_);
  pub_competition_path_->publish(competition_path_msg_);
  pub_markers_->publish(markers_msg_);

  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / supervisor_hz_));
  timer_ = create_wall_timer(period, std::bind(&MissionSemanticsNode::tick_, this));

  RCLCPP_INFO(get_logger(),
      "mission_semantics_node: %.1f Hz, %zu mission nodes, graph %zu nodes, "
      "%zu validation errors (mission_valid=%s)",
      supervisor_hz_, mission_path_.size(), graph_.n_nodes(),
      validation_errors_.size(), mission_valid_ ? "true" : "false");
}

// ---- YAML loaders ----------------------------------------------------------

bool MissionSemanticsNode::load_mission_(const std::string & path)
{
  try {
    YAML::Node y = YAML::LoadFile(path);
    auto m = y["mission"];
    if (!m || !m["path"]) {
      RCLCPP_ERROR(get_logger(), "MISSION: missing mission.path in %s", path.c_str());
      return false;
    }
    for (const auto & n : m["path"]) {
      mission_path_.push_back(yaml_node_id(n));
    }
    RCLCPP_INFO(get_logger(),
                "MISSION: loaded path with %zu nodes (start=%s goal=%s)",
                mission_path_.size(),
                m["start_node"] ? m["start_node"].as<std::string>().c_str() : "?",
                m["goal_node"]  ? m["goal_node"] .as<std::string>().c_str() : "?");
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MISSION: %s: %s", path.c_str(), e.what());
    return false;
  }
}

bool MissionSemanticsNode::load_semantics_(const std::string & path)
{
  try {
    YAML::Node y = YAML::LoadFile(path);
    if (y["default_region"]) default_region_ = y["default_region"].as<std::string>();
    auto regs = y["regions"];
    if (!regs) {
      RCLCPP_WARN(get_logger(), "MISSION: %s missing 'regions:'", path.c_str());
      return false;
    }
    for (auto it = regs.begin(); it != regs.end(); ++it) {
      const std::string region = it->first.as<std::string>();
      const YAML::Node body = it->second;
      if (body["nodes"]) {
        for (const auto & n : body["nodes"])
          node_region_[yaml_node_id(n)] = region;
      }
      if (body["node_ranges"]) {
        for (const auto & range : body["node_ranges"]) {
          if (range.size() < 2) continue;
          int lo = range[0].as<int>();
          int hi = range[1].as<int>();
          if (lo > hi) std::swap(lo, hi);
          for (int i = lo; i <= hi; ++i) node_region_[std::to_string(i)] = region;
        }
      }
    }
    RCLCPP_INFO(get_logger(), "MISSION: semantics loaded — %zu tagged nodes",
                node_region_.size());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MISSION: %s: %s", path.c_str(), e.what());
    return false;
  }
}

bool MissionSemanticsNode::load_speed_profiles_(const std::string & path)
{
  try {
    YAML::Node y = YAML::LoadFile(path);
    if (y["v_floor"]) v_floor_ = y["v_floor"].as<double>();
    if (y["v_cap"])   v_cap_   = y["v_cap"]  .as<double>();
    auto sp = y["speed_profiles"];
    if (!sp) {
      RCLCPP_WARN(get_logger(), "MISSION: %s missing 'speed_profiles:'", path.c_str());
      return false;
    }
    for (auto it = sp.begin(); it != sp.end(); ++it) {
      const std::string region = it->first.as<std::string>();
      RegionDispatch & d = region_dispatch_[region];   // create if absent
      d.speed = std::clamp(it->second.as<double>(), v_floor_, v_cap_);
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MISSION: %s: %s", path.c_str(), e.what());
    return false;
  }
}

bool MissionSemanticsNode::load_risk_zones_(const std::string & path)
{
  try {
    YAML::Node y = YAML::LoadFile(path);
    if (y["loc_low_thresh"])   loc_low_thresh_   = y["loc_low_thresh"].as<double>();
    if (y["graceful_decay_s"]) graceful_decay_s_ = y["graceful_decay_s"].as<double>();
    auto bw = y["blend_weights"];
    if (!bw) {
      RCLCPP_WARN(get_logger(), "MISSION: %s missing 'blend_weights:'", path.c_str());
      return false;
    }
    for (auto it = bw.begin(); it != bw.end(); ++it) {
      const std::string region = it->first.as<std::string>();
      const YAML::Node b = it->second;
      BlendWeights w;
      if (b["loc"])     w.loc     = b["loc"].as<double>();
      if (b["lane"])    w.lane    = b["lane"].as<double>();
      if (b["graph"])   w.graph   = b["graph"].as<double>();
      if (b["imu_yaw"]) w.imu_yaw = b["imu_yaw"].as<double>();
      region_blend_[region] = w;
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MISSION: %s: %s", path.c_str(), e.what());
    return false;
  }
}

bool MissionSemanticsNode::load_behavior_overrides_(const std::string & path)
{
  try {
    YAML::Node y = YAML::LoadFile(path);
    auto ov = y["overrides"];
    if (!ov) {
      RCLCPP_WARN(get_logger(), "MISSION: %s missing 'overrides:'", path.c_str());
      return false;
    }
    for (auto it = ov.begin(); it != ov.end(); ++it) {
      const std::string region = it->first.as<std::string>();
      const YAML::Node row = it->second;
      RegionDispatch & d = region_dispatch_[region];   // merge with speed_profiles
      if (row["maneuver"]) d.maneuver = row["maneuver"].as<std::string>();
      if (row["speed"])    d.speed    = std::clamp(row["speed"].as<double>(),
                                                    v_floor_, v_cap_);
      if (row["nav_flag"]) d.nav_flag = row["nav_flag"].as<std::string>();
    }
    if (y["unknown_region"]) {
      const YAML::Node row = y["unknown_region"];
      if (row["maneuver"]) unknown_dispatch_.maneuver = row["maneuver"].as<std::string>();
      if (row["speed"])    unknown_dispatch_.speed    = row["speed"].as<double>();
      if (row["nav_flag"]) unknown_dispatch_.nav_flag = row["nav_flag"].as<std::string>();
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MISSION: %s: %s", path.c_str(), e.what());
    return false;
  }
}

// ---- Validation ------------------------------------------------------------

void MissionSemanticsNode::validate_mission_()
{
  validation_errors_.clear();
  if (mission_path_.empty()) {
    validation_errors_.emplace_back("mission_path is empty");
    return;
  }
  for (std::size_t i = 0; i < mission_path_.size(); ++i) {
    if (!graph_.has_node(mission_path_[i])) {
      std::ostringstream os;
      os << "node[" << i << "] id=" << mission_path_[i] << " not in graph";
      validation_errors_.push_back(os.str());
    }
  }
  for (std::size_t i = 1; i < mission_path_.size(); ++i) {
    const auto & a = mission_path_[i - 1];
    const auto & b = mission_path_[i];
    if (!graph_.has_node(a) || !graph_.has_node(b)) continue;  // already flagged
    if (!graph_.has_edge(a, b)) {
      std::ostringstream os;
      os << "missing directed edge " << a << " → " << b << " at idx " << i;
      validation_errors_.push_back(os.str());
    }
  }
  mission_valid_ = validation_errors_.empty();
  if (!mission_valid_) {
    // Log up to first 10 errors — the rest still get reflected on /rosout
    // through individual WARN below, but the summary is enough for race-day.
    RCLCPP_WARN(get_logger(),
        "MISSION VALIDATION: %zu issues found — race-day will run with "
        "best-effort path. Behavior planner will still cope via /navigation/flags.",
        validation_errors_.size());
    const std::size_t cap = std::min<std::size_t>(10, validation_errors_.size());
    for (std::size_t i = 0; i < cap; ++i)
      RCLCPP_WARN(get_logger(), "  [validation] %s", validation_errors_[i].c_str());
  } else {
    RCLCPP_INFO(get_logger(), "MISSION VALIDATION: PASS (%zu nodes, all edges legal)",
                mission_path_.size());
  }
}

// ---- Path / marker construction --------------------------------------------

void MissionSemanticsNode::build_competition_path_()
{
  competition_path_msg_.header.frame_id = "map";
  competition_path_msg_.header.stamp = this->now();
  competition_path_msg_.poses.clear();
  competition_path_msg_.poses.reserve(mission_path_.size());
  for (std::size_t i = 0; i < mission_path_.size(); ++i) {
    const auto * n = graph_.node(mission_path_[i]);
    if (n == nullptr) continue;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = competition_path_msg_.header.stamp;
    ps.pose.position.x = n->x;
    ps.pose.position.y = n->y;
    ps.pose.position.z = 0.0;
    // Tangent yaw towards next node (last point inherits previous tangent).
    double yaw = 0.0;
    if (i + 1 < mission_path_.size()) {
      const auto * nn = graph_.node(mission_path_[i + 1]);
      if (nn) yaw = std::atan2(nn->y - n->y, nn->x - n->x);
    } else if (!competition_path_msg_.poses.empty()) {
      const auto & prev = competition_path_msg_.poses.back().pose;
      yaw = std::atan2(n->y - prev.position.y, n->x - prev.position.x);
    }
    const double half = yaw * 0.5;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = std::sin(half);
    ps.pose.orientation.w = std::cos(half);
    competition_path_msg_.poses.push_back(ps);
  }
  RCLCPP_INFO(get_logger(),
              "MISSION path built — %zu poses in 'map' frame",
              competition_path_msg_.poses.size());
}

void MissionSemanticsNode::build_marker_array_()
{
  markers_msg_.markers.clear();
  if (competition_path_msg_.poses.empty()) return;

  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp = this->now();
  line.ns = "mission_path";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.06;            // 6 cm wide line
  line.pose.orientation.w = 1.0;
  line.color.a = 1.0;

  // Per-point colour by region.
  auto color_for_region = [](const std::string & region)
      -> std_msgs::msg::ColorRGBA
  {
    std_msgs::msg::ColorRGBA c; c.a = 1.0;
    if (region == "highway" || region == "highway_entry" || region == "highway_exit") {
      c.r = 1.0; c.g = 0.55; c.b = 0.0;   // orange
    } else if (region == "low_visibility" || region == "tunnel" || region == "no_lane") {
      c.r = 1.0; c.g = 0.0;  c.b = 0.0;   // red
    } else if (region == "crosswalk" || region == "intersection" || region == "roundabout") {
      c.r = 1.0; c.g = 1.0;  c.b = 0.0;   // yellow
    } else if (region == "parking") {
      c.r = 0.0; c.g = 0.7;  c.b = 1.0;   // light blue (parking spot pictogram)
    } else {
      c.r = 0.0; c.g = 1.0;  c.b = 0.0;   // green = normal
    }
    return c;
  };

  for (std::size_t i = 0; i < competition_path_msg_.poses.size(); ++i) {
    const auto & p = competition_path_msg_.poses[i].pose.position;
    geometry_msgs::msg::Point pt;
    pt.x = p.x; pt.y = p.y; pt.z = 0.02;
    line.points.push_back(pt);
    const std::string & region = region_of_node_(mission_path_[i]);
    line.colors.push_back(color_for_region(region));
  }
  markers_msg_.markers.push_back(line);

  // Highlight start and goal.
  auto sphere = [&](int id, double x, double y, double r, double g, double b,
                    const std::string & ns)
  {
    visualization_msgs::msg::Marker s;
    s.header = line.header;
    s.ns = ns; s.id = id;
    s.type = visualization_msgs::msg::Marker::SPHERE;
    s.action = visualization_msgs::msg::Marker::ADD;
    s.scale.x = s.scale.y = s.scale.z = 0.20;
    s.pose.position.x = x; s.pose.position.y = y; s.pose.position.z = 0.10;
    s.pose.orientation.w = 1.0;
    s.color.a = 1.0; s.color.r = r; s.color.g = g; s.color.b = b;
    markers_msg_.markers.push_back(s);
  };
  const auto * start = graph_.node(mission_path_.front());
  const auto * goal  = graph_.node(mission_path_.back());
  if (start) sphere(100, start->x, start->y, 0.0, 1.0, 0.0, "mission_start");
  if (goal)  sphere(101, goal->x,  goal->y,  1.0, 0.0, 1.0, "mission_goal");
}

// ---- Region / dispatch lookups ---------------------------------------------

const std::string & MissionSemanticsNode::region_of_node_(const std::string & node_id) const
{
  auto it = node_region_.find(node_id);
  if (it != node_region_.end()) return it->second;
  return default_region_;
}

const RegionDispatch & MissionSemanticsNode::dispatch_of_region_(const std::string & region) const
{
  auto it = region_dispatch_.find(region);
  if (it != region_dispatch_.end()) return it->second;
  return unknown_dispatch_;
}

const BlendWeights & MissionSemanticsNode::blend_of_region_(const std::string & region) const
{
  static const BlendWeights kDefault{};
  auto it = region_blend_.find(region);
  if (it != region_blend_.end()) return it->second;
  return kDefault;
}

// ---- Callbacks --------------------------------------------------------------

void MissionSemanticsNode::on_state_(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  state_x_   = msg->pose.position.x;
  state_y_   = msg->pose.position.y;
  state_conf_ = msg->confidence;
  have_state_ = true;
  last_state_t_ = this->now();
}

void MissionSemanticsNode::on_imu_conf_(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  imu_conf_ = msg->data;
}

// ---- Tick ------------------------------------------------------------------

void MissionSemanticsNode::tick_()
{
  // 1) Republish the mission path (Transient-local QoS, but the timer
  //    heartbeat covers any RTPS resub edge case).
  competition_path_msg_.header.stamp = this->now();
  pub_global_path_->publish(competition_path_msg_);
  pub_competition_path_->publish(competition_path_msg_);
  pub_markers_->publish(markers_msg_);

  // 2) If we don't yet have a state, still pre-arm with starting-node info.
  std::size_t idx = current_mission_idx_;
  std::string region = default_region_;
  bool have_state_now = false;
  double sx = 0.0, sy = 0.0;
  double imu_c = 1.0, state_c = 0.0;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    have_state_now = have_state_;
    sx = state_x_; sy = state_y_;
    imu_c = imu_conf_;
    state_c = state_conf_;
  }

  if (have_state_now && !mission_path_.empty()) {
    // Nearest mission-node search (O(N), N≤500 → trivial).
    double best_d2 = std::numeric_limits<double>::infinity();
    std::size_t best_idx = current_mission_idx_;
    for (std::size_t i = 0; i < mission_path_.size(); ++i) {
      const auto * n = graph_.node(mission_path_[i]);
      if (!n) continue;
      const double dx = sx - n->x, dy = sy - n->y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_idx = i; }
    }
    // Hysteresis. Mission progress is monotonic, AND we resist flicker
    // between adjacent close nodes (e.g. 399 ↔ 400):
    //
    //   (i)  Backwards snaps are ignored — current_mission_idx_ never regresses.
    //   (ii) Forward commit requires either
    //          (a) the new index's distance² be at least
    //              hys_distance_margin_m_² better than current's, OR
    //          (b) the new index has been the closest for ≥ hys_persist_s_.
    //
    // Without (ii) the index would jump every tick whenever the vehicle
    // crosses the bisector between two adjacent mission nodes.
    const auto * cur_n = graph_.node(mission_path_[current_mission_idx_]);
    const double cur_d2 = cur_n
        ? std::pow(sx - cur_n->x, 2.0) + std::pow(sy - cur_n->y, 2.0)
        : std::numeric_limits<double>::infinity();
    const double margin2 = hys_distance_margin_m_ * hys_distance_margin_m_;

    bool commit = false;
    if (best_idx > current_mission_idx_) {
      const bool distance_decisive = (cur_d2 - best_d2) > margin2;
      if (distance_decisive) {
        commit = true;
      } else {
        // Persistence path: same candidate held for long enough?
        if (best_idx != hys_candidate_idx_) {
          hys_candidate_idx_ = best_idx;
          hys_candidate_t_   = this->now();
        } else {
          const double held_s = (this->now() - hys_candidate_t_).seconds();
          if (held_s >= hys_persist_s_) commit = true;
        }
      }
    } else {
      // best_idx <= current_mission_idx_ — clear pending candidate.
      hys_candidate_idx_ = current_mission_idx_;
      hys_candidate_t_   = this->now();
    }
    if (commit) {
      current_mission_idx_ = best_idx;
      hys_candidate_idx_   = best_idx;
      hys_candidate_t_     = this->now();
    }
    idx    = current_mission_idx_;
    region = region_of_node_(mission_path_[idx]);
  } else if (!mission_path_.empty()) {
    // Pre-state: advertise mission start.
    idx = 0;
    region = region_of_node_(mission_path_.front());
  }
  current_region_ = region;

  // 3) Dispatch row for the region.
  const RegionDispatch & disp = dispatch_of_region_(region);

  // 4) Blended localization confidence — region-aware weighted sum, with
  //    graceful decay when imu_conf collapses.
  const BlendWeights & bw = blend_of_region_(region);
  // The three signals we treat as 0..1:
  //   loc      ← state.confidence
  //   lane     ← imu_conf (proxy: when IMU is solid, lane fit is more trustable)
  //   graph    ← always 1.0 if we have a region tag (it's a static prior)
  //   imu_yaw  ← imu_conf
  const double graph_signal = node_region_.count(have_state_now ? mission_path_[idx] : std::string())
                              ? 1.0 : 0.8;
  const double w_sum = bw.loc + bw.lane + bw.graph + bw.imu_yaw;
  double blended = (w_sum > 1e-6)
      ? (bw.loc * state_c + bw.lane * imu_c + bw.graph * graph_signal + bw.imu_yaw * imu_c) / w_sum
      : state_c;
  if (imu_c < loc_low_thresh_) {
    // Graceful decay: hold last_good_blended_conf_ for graceful_decay_s_ seconds.
    if ((this->now() - last_state_t_).seconds() < graceful_decay_s_) {
      blended = last_good_blended_conf_;
    }
  } else {
    last_good_blended_conf_ = blended;
  }
  blended = std::clamp(blended, 0.0, 1.0);

  // 5) Score estimate — simplistic: progress fraction * 100 minus
  //    region-mismatch penalties (we have none yet). Real BFMC scoring is
  //    done by the judges; this is just a live operator sanity number.
  const double progress = mission_path_.empty()
      ? 0.0
      : static_cast<double>(idx) / static_cast<double>(mission_path_.size() - 1);
  const double score_est = 100.0 * progress;

  // 6) Publish everything.
  std_msgs::msg::Int32   nid;     nid.data = mission_path_.empty()
                                            ? -1
                                            : std::atoi(mission_path_[idx].c_str());
  std_msgs::msg::String  nreg;    nreg.data = region;
  std_msgs::msg::String  nbeh;    nbeh.data = disp.maneuver;
  std_msgs::msg::Float64 nspeed;  nspeed.data = disp.speed;
  std_msgs::msg::Float64 nprog;   nprog.data = progress;
  std_msgs::msg::Float64 nscore;  nscore.data = score_est;
  std_msgs::msg::Float64 nloc;    nloc.data = blended;
  std_msgs::msg::String  nflag;   nflag.data = disp.nav_flag;

  pub_current_node_->publish(nid);
  pub_current_region_->publish(nreg);
  pub_current_behavior_->publish(nbeh);
  pub_current_speed_->publish(nspeed);
  pub_progress_->publish(nprog);
  pub_score_est_->publish(nscore);
  pub_loc_conf_->publish(nloc);

  // /navigation/flags only published on change (avoid spamming behavior_planner).
  if (nflag.data != last_nav_flag_published_) {
    pub_nav_flag_->publish(nflag);
    last_nav_flag_published_ = nflag.data;
    RCLCPP_INFO(get_logger(),
                "MISSION region change → %s (nav_flag='%s' speed=%.2f beh=%s)",
                region.c_str(), nflag.data.c_str(), disp.speed,
                disp.maneuver.c_str());
  }
}

}  // namespace mission_supervisor_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_supervisor_cpp::MissionSemanticsNode>());
  rclcpp::shutdown();
  return 0;
}
