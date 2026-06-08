#include "behavior_planner_cpp/behavior_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace behavior_planner_cpp
{

namespace
{

bool class_matches(const std::string & needle,
                   const std::vector<std::string> & haystack)
{
  for (const auto & h : haystack) if (h == needle) return true;
  return false;
}

double yaw_from_quat(double qz, double qw)
{
  return 2.0 * std::atan2(qz, qw);
}

double euclid_to_obj(const bfmc_msgs::msg::DetectedObject & o)
{
  return std::hypot(o.position.x, o.position.y);
}

}  // namespace

BehaviorPlannerNode::BehaviorPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("behavior_planner_node", options)
{}

/* =========================== Lifecycle ============================ */

CallbackReturn BehaviorPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  // ---- Tick rate ---------------------------------------------------------
  tick_period_s_ = declare_parameter<double>("tick_period_s", tick_period_s_);

  // ---- FsmTuning ---------------------------------------------------------
  tuning_.low_imu_conf_thresh         = declare_parameter<double>("low_imu_conf_thresh",         tuning_.low_imu_conf_thresh);
  tuning_.low_state_conf_thresh       = declare_parameter<double>("low_state_conf_thresh",       tuning_.low_state_conf_thresh);
  tuning_.pedestrian_dist_thresh      = declare_parameter<double>("pedestrian_dist_thresh",      tuning_.pedestrian_dist_thresh);
  tuning_.traffic_light_dist_thresh   = declare_parameter<double>("traffic_light_dist_thresh",   tuning_.traffic_light_dist_thresh);
  tuning_.stop_sign_dist_thresh       = declare_parameter<double>("stop_sign_dist_thresh",       tuning_.stop_sign_dist_thresh);
  tuning_.roadblock_dist_thresh       = declare_parameter<double>("roadblock_dist_thresh",       tuning_.roadblock_dist_thresh);
  tuning_.vehicle_follow_dist_thresh  = declare_parameter<double>("vehicle_follow_dist_thresh",  tuning_.vehicle_follow_dist_thresh);
  tuning_.crosswalk_dist_thresh       = declare_parameter<double>("crosswalk_dist_thresh",       tuning_.crosswalk_dist_thresh);
  tuning_.no_entry_dist_thresh        = declare_parameter<double>("no_entry_dist_thresh",        tuning_.no_entry_dist_thresh);
  tuning_.parking_dist_thresh         = declare_parameter<double>("parking_dist_thresh",         tuning_.parking_dist_thresh);
  tuning_.roundabout_dist_thresh      = declare_parameter<double>("roundabout_dist_thresh",      tuning_.roundabout_dist_thresh);

  tuning_.city_speed        = declare_parameter<double>("city_speed",        tuning_.city_speed);
  tuning_.highway_speed     = declare_parameter<double>("highway_speed",     tuning_.highway_speed);
  tuning_.parking_speed     = declare_parameter<double>("parking_speed",     tuning_.parking_speed);
  tuning_.crosswalk_speed   = declare_parameter<double>("crosswalk_speed",   tuning_.crosswalk_speed);
  tuning_.follow_slow_speed = declare_parameter<double>("follow_slow_speed", tuning_.follow_slow_speed);
  tuning_.tailing_min_speed = declare_parameter<double>("tailing_min_speed", tuning_.tailing_min_speed);
  tuning_.turn_speed        = declare_parameter<double>("turn_speed",        tuning_.turn_speed);
  tuning_.roundabout_speed  = declare_parameter<double>("roundabout_speed",  tuning_.roundabout_speed);
  tuning_.turn_speed_priority = declare_parameter<double>("turn_speed_priority", tuning_.turn_speed_priority);

  tuning_.stop_sign_wait_s  = declare_parameter<double>("stop_sign_wait_s",  tuning_.stop_sign_wait_s);
  tuning_.highway_ramp_s    = declare_parameter<double>("highway_ramp_s",    tuning_.highway_ramp_s);

  tuning_.a_max       = declare_parameter<double>("a_max",       tuning_.a_max);
  tuning_.t_react     = declare_parameter<double>("t_react",     tuning_.t_react);
  tuning_.stop_margin = declare_parameter<double>("stop_margin", tuning_.stop_margin);

  // ---- Class-name match lists -------------------------------------------
  classes_pedestrian_    = declare_parameter<std::vector<std::string>>("classes_pedestrian",    classes_pedestrian_);
  classes_traffic_light_ = declare_parameter<std::vector<std::string>>("classes_traffic_light", classes_traffic_light_);
  classes_stop_sign_     = declare_parameter<std::vector<std::string>>("classes_stop_sign",     classes_stop_sign_);
  classes_roadblock_     = declare_parameter<std::vector<std::string>>("classes_roadblock",     classes_roadblock_);
  classes_vehicle_       = declare_parameter<std::vector<std::string>>("classes_vehicle",       classes_vehicle_);
  classes_crosswalk_     = declare_parameter<std::vector<std::string>>("classes_crosswalk",     classes_crosswalk_);
  classes_parking_       = declare_parameter<std::vector<std::string>>("classes_parking",       classes_parking_);
  classes_roundabout_    = declare_parameter<std::vector<std::string>>("classes_roundabout",    classes_roundabout_);
  classes_highway_entry_ = declare_parameter<std::vector<std::string>>("classes_highway_entry", classes_highway_entry_);
  classes_highway_exit_  = declare_parameter<std::vector<std::string>>("classes_highway_exit",  classes_highway_exit_);
  classes_no_entry_      = declare_parameter<std::vector<std::string>>("classes_no_entry",      classes_no_entry_);
  classes_priority_      = declare_parameter<std::vector<std::string>>("classes_priority",      classes_priority_);
  classes_onewayroad_    = declare_parameter<std::vector<std::string>>("classes_onewayroad",    classes_onewayroad_);

  // ---- DetectionTimer hysteresis params ---------------------------------
  pedestrian_confirm_s_ = declare_parameter<double>("pedestrian_confirm_s", pedestrian_confirm_s_);
  pedestrian_clear_s_   = declare_parameter<double>("pedestrian_clear_s",   pedestrian_clear_s_);
  vehicle_confirm_s_    = declare_parameter<double>("vehicle_confirm_s",    vehicle_confirm_s_);
  vehicle_clear_s_      = declare_parameter<double>("vehicle_clear_s",      vehicle_clear_s_);
  sign_confirm_s_       = declare_parameter<double>("sign_confirm_s",       sign_confirm_s_);
  sign_clear_s_         = declare_parameter<double>("sign_clear_s",         sign_clear_s_);
  roadblock_confirm_s_  = declare_parameter<double>("roadblock_confirm_s",  roadblock_confirm_s_);
  roadblock_clear_s_    = declare_parameter<double>("roadblock_clear_s",    roadblock_clear_s_);

  // ---- Stale gates ------------------------------------------------------
  stale_state_s_     = declare_parameter<double>("stale_state_s",     stale_state_s_);
  stale_lane_s_      = declare_parameter<double>("stale_lane_s",      stale_lane_s_);
  stale_objects_s_   = declare_parameter<double>("stale_objects_s",   stale_objects_s_);
  stale_path_s_      = declare_parameter<double>("stale_path_s",      stale_path_s_);
  stale_freespace_s_ = declare_parameter<double>("stale_freespace_s", stale_freespace_s_);

  // ---- FreeSpace + lane overtake-check params ---------------------------
  freespace_left_col_start_ = declare_parameter<int>("freespace_left_col_start", freespace_left_col_start_);
  freespace_left_col_end_   = declare_parameter<int>("freespace_left_col_end",   freespace_left_col_end_);
  freespace_clear_thresh_   = declare_parameter<double>("freespace_clear_threshold", freespace_clear_thresh_);
  lane_dashed_thresh_       = declare_parameter<double>("lane_dashed_thresh",         lane_dashed_thresh_);

  // ---- CSV logging ------------------------------------------------------
  enable_csv_logging_ = declare_parameter<bool>("enable_csv_logging", enable_csv_logging_);
  csv_log_dir_        = declare_parameter<std::string>("csv_log_dir",  csv_log_dir_);

  // ---- Build per-class DetectionTimers ---------------------------------
  det_timers_.clear();
  det_timers_.emplace("pedestrian",  DetectionTimer(pedestrian_confirm_s_, pedestrian_clear_s_));
  det_timers_.emplace("vehicle",     DetectionTimer(vehicle_confirm_s_,    vehicle_clear_s_));
  det_timers_.emplace("trafficlight",DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("stop",        DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("crosswalk",   DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("roadblock",   DetectionTimer(roadblock_confirm_s_,  roadblock_clear_s_));
  det_timers_.emplace("parking",     DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("roundabout",  DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("highway_entry", DetectionTimer(sign_confirm_s_,     sign_clear_s_));
  det_timers_.emplace("highway_exit",  DetectionTimer(sign_confirm_s_,     sign_clear_s_));
  // Race-day finals: no_entry (regulatory hard stop), priority (right-of-way
  // modifier at intersections), onewayroad (informational).
  det_timers_.emplace("no_entry",    DetectionTimer(roadblock_confirm_s_,  roadblock_clear_s_));
  det_timers_.emplace("priority",    DetectionTimer(sign_confirm_s_,       sign_clear_s_));
  det_timers_.emplace("onewayroad",  DetectionTimer(sign_confirm_s_,       sign_clear_s_));

  // ---- ROS interfaces ---------------------------------------------------
  auto rel_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  auto be_qos  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  sub_state_   = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", rel_qos,
    std::bind(&BehaviorPlannerNode::on_state, this, std::placeholders::_1));
  sub_objects_ = create_subscription<bfmc_msgs::msg::ObjectArray>(
    "/perception/tracked_objects", rel_qos,
    std::bind(&BehaviorPlannerNode::on_objects, this, std::placeholders::_1));
  sub_lane_    = create_subscription<bfmc_msgs::msg::LaneGeometry>(
    "/perception/lane_geometry", be_qos,
    std::bind(&BehaviorPlannerNode::on_lane, this, std::placeholders::_1));
  sub_freespace_ = create_subscription<bfmc_msgs::msg::FreeSpace>(
    "/perception/free_space", be_qos,
    std::bind(&BehaviorPlannerNode::on_freespace, this, std::placeholders::_1));
  sub_path_    = create_subscription<nav_msgs::msg::Path>(
    "/planning/global_path", rel_qos,
    std::bind(&BehaviorPlannerNode::on_path, this, std::placeholders::_1));
  sub_imu_conf_ = create_subscription<std_msgs::msg::Float64>(
    "/fusion/imu_confidence", be_qos,
    std::bind(&BehaviorPlannerNode::on_imu_conf, this, std::placeholders::_1));
  sub_emergency_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/emergency", rel_qos,
    std::bind(&BehaviorPlannerNode::on_emergency, this, std::placeholders::_1));
  sub_nav_flag_ = create_subscription<std_msgs::msg::String>(
    "/navigation/flags", rel_qos,
    std::bind(&BehaviorPlannerNode::on_nav_flag, this, std::placeholders::_1));
  sub_semaphore_ = create_subscription<std_msgs::msg::String>(
    "/v2x/semaphores", rel_qos,
    std::bind(&BehaviorPlannerNode::on_semaphore, this, std::placeholders::_1));
  // Mission-supervisor speed override. When connected, LANE_KEEP / FOLLOW
  // (overtake-intent) target speeds use the YAML speed profile (0.45 city
  // / 0.70 highway / etc.). Safety-priority maneuvers ignore this.
  sub_mission_speed_ = create_subscription<std_msgs::msg::Float64>(
    "/planning/current_speed_limit", rel_qos,
    std::bind(&BehaviorPlannerNode::on_mission_speed_limit, this, std::placeholders::_1));

  pub_lateral_offset_ = create_publisher<std_msgs::msg::Float32>(
      "/planning/lateral_offset", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  pub_behavior_     = create_publisher<std_msgs::msg::String>(
    "/planning/behavior_state", rel_qos);
  pub_target_speed_ = create_publisher<std_msgs::msg::Float32>(
    "/planning/target_speed",   rel_qos);

  pub_dbg_state_      = create_publisher<std_msgs::msg::String>("/planning/debug/current_state",   be_qos);
  pub_dbg_node_       = create_publisher<std_msgs::msg::String>("/planning/debug/current_node",    be_qos);
  pub_dbg_goal_       = create_publisher<std_msgs::msg::String>("/planning/debug/current_goal",    be_qos);
  pub_dbg_dev_        = create_publisher<std_msgs::msg::Float64>("/planning/debug/path_deviation", be_qos);
  pub_dbg_next_       = create_publisher<std_msgs::msg::String>("/planning/debug/next_maneuver",   be_qos);
  pub_dbg_transition_ = create_publisher<std_msgs::msg::String>("/planning/debug/fsm_transition",  be_qos);

  // ---- CSV logger setup ------------------------------------------------
  if (enable_csv_logging_) {
    if (open_csv_logs_()) {
      RCLCPP_INFO(get_logger(),
        "behavior_planner_cpp: CSV logging ENABLED → %s/fsm.csv",
        csv_log_dir_.c_str());
    } else {
      enable_csv_logging_ = false;
      RCLCPP_ERROR(get_logger(),
        "behavior_planner_cpp: CSV logging DISABLED — cannot open %s",
        csv_log_dir_.c_str());
    }
  }

  RCLCPP_INFO(get_logger(),
    "behavior_planner_cpp configured: tick=%.0fms, city/hwy/park/cw=%.2f/%.2f/%.2f/%.2f m/s, "
    "stop_wait=%.1fs, hwy_ramp=%.1fs, classes match BFMC detector (Phase Final)",
    tick_period_s_ * 1000.0,
    tuning_.city_speed, tuning_.highway_speed, tuning_.parking_speed, tuning_.crosswalk_speed,
    tuning_.stop_sign_wait_s, tuning_.highway_ramp_s);
  return CallbackReturn::SUCCESS;
}

CallbackReturn BehaviorPlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  pub_behavior_      ->on_activate();
  pub_target_speed_  ->on_activate();
  pub_lateral_offset_->on_activate();
  active_ = true;
  fsm_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(tick_period_s_ * 1000.0)),
    std::bind(&BehaviorPlannerNode::fsm_tick, this));
  return CallbackReturn::SUCCESS;
}
CallbackReturn BehaviorPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  fsm_timer_.reset();
  pub_behavior_      ->on_deactivate();
  pub_target_speed_  ->on_deactivate();
  pub_lateral_offset_->on_deactivate();
  return CallbackReturn::SUCCESS;
}
CallbackReturn BehaviorPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  sub_state_.reset(); sub_objects_.reset(); sub_lane_.reset();
  sub_path_.reset();  sub_imu_conf_.reset(); sub_emergency_.reset();
  sub_freespace_.reset(); sub_nav_flag_.reset(); sub_semaphore_.reset();
  sub_mission_speed_.reset();
  pub_behavior_.reset(); pub_target_speed_.reset();
  pub_dbg_state_.reset(); pub_dbg_node_.reset(); pub_dbg_goal_.reset();
  pub_dbg_dev_.reset();   pub_dbg_next_.reset(); pub_dbg_transition_.reset();
  close_csv_logs_();
  return CallbackReturn::SUCCESS;
}
CallbackReturn BehaviorPlannerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  active_ = false;
  close_csv_logs_();
  return CallbackReturn::SUCCESS;
}

/* =========================== Callbacks ============================ */

void BehaviorPlannerNode::on_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_state_     = *msg;
  have_state_       = true;
  last_state_time_  = this->now();
}
void BehaviorPlannerNode::on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_objects_    = *msg;
  last_objects_time_ = this->now();
}
void BehaviorPlannerNode::on_lane(const bfmc_msgs::msg::LaneGeometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_lane_    = *msg;
  last_lane_time_ = this->now();
}
void BehaviorPlannerNode::on_freespace(const bfmc_msgs::msg::FreeSpace::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_freespace_    = *msg;
  last_freespace_time_ = this->now();
}
void BehaviorPlannerNode::on_path(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  latest_path_    = *msg;
  last_path_time_ = this->now();
}
void BehaviorPlannerNode::on_imu_conf(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  last_imu_conf_ = msg->data;
}
void BehaviorPlannerNode::on_emergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  emergency_flag_ = msg->data;
}
void BehaviorPlannerNode::on_nav_flag(const std_msgs::msg::String::SharedPtr msg)
{
  // Lower-cased & trimmed; nav publisher controls exact strings.
  std::lock_guard<std::mutex> lk(cache_mtx_);
  nav_flag_ = msg->data;
  std::transform(nav_flag_.begin(), nav_flag_.end(), nav_flag_.begin(),
                 [](unsigned char c){ return std::tolower(c); });
}
void BehaviorPlannerNode::on_mission_speed_limit(const std_msgs::msg::Float64::SharedPtr msg)
{
  // Atomic single-writer/single-reader. -1.0 = "no override yet"; 0 or
  // negative from upstream is ignored (treated as no-override) so a bug in
  // mission_supervisor cannot drive the car to 0 m/s.
  if (msg->data > 0.0) mission_speed_limit_.store(msg->data, std::memory_order_relaxed);
}

void BehaviorPlannerNode::on_semaphore(const std_msgs::msg::String::SharedPtr msg)
{
  // Minimal JSON parser — we want id + state. Format is:
  //   {"id":"<id>","state":"<red|yellow|green>","x":..,"y":..}
  // (v2x_bridge publishes exactly this — see protocol in v2x/udpListener.py.)
  std::lock_guard<std::mutex> lk(cache_mtx_);
  const std::string & s = msg->data;
  auto find_field = [&](const std::string & key) -> std::string {
    const std::string pat = "\"" + key + "\"";
    auto pos = s.find(pat);
    if (pos == std::string::npos) return "";
    pos = s.find(':', pos);
    if (pos == std::string::npos) return "";
    // skip whitespace + opening quote (if any)
    while (++pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) {}
    if (pos < s.size() && s[pos] == '"') {
      auto end = s.find('"', pos + 1);
      return (end == std::string::npos) ? "" : s.substr(pos + 1, end - pos - 1);
    }
    // numeric / unquoted token
    auto end = s.find_first_of(",}", pos);
    return s.substr(pos, end - pos);
  };
  const std::string id    = find_field("id");
  const std::string state = find_field("state");
  if (!id.empty() && !state.empty()) {
    std::string st = state;
    std::transform(st.begin(), st.end(), st.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    semaphore_state_[id] = st;
  }
}

/* =========================== FSM tick ============================ */

bool BehaviorPlannerNode::freespace_left_clear_(
    const bfmc_msgs::msg::FreeSpace & fs,
    int col_start, int col_end, double threshold) const
{
  const std::size_t width = fs.width, height = fs.height;
  if (width == 0 || height == 0) return false;
  const std::size_t ce = std::min<std::size_t>(col_end, width);
  std::size_t occupied = 0, total = 0;
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t col = col_start; col < ce; ++col) {
      const std::size_t idx = row * width + col;
      if (idx < fs.occupancy_grid.size()) {
        ++total;
        if (static_cast<double>(fs.occupancy_grid[idx]) > threshold) ++occupied;
      }
    }
  }
  if (total == 0) return false;
  return (static_cast<double>(occupied) / total) < 0.15;
}

std::optional<char> BehaviorPlannerNode::derive_intersection_turn_() const
{
  if (latest_path_.poses.size() < 3 || !have_state_) return std::nullopt;
  if (last_path_time_.nanoseconds() == 0) return std::nullopt;
  if ((this->now() - last_path_time_).seconds() > stale_path_s_) return std::nullopt;

  const double vx  = latest_state_.pose.position.x;
  const double vy  = latest_state_.pose.position.y;
  const double yaw = yaw_from_quat(latest_state_.pose.orientation.z,
                                    latest_state_.pose.orientation.w);
  std::size_t nearest = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < latest_path_.poses.size(); ++i) {
    const double dx = latest_path_.poses[i].pose.position.x - vx;
    const double dy = latest_path_.poses[i].pose.position.y - vy;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; nearest = i; }
  }
  if (nearest + 2 >= latest_path_.poses.size()) return std::nullopt;

  const auto & a = latest_path_.poses[nearest    ].pose.position;
  const auto & b = latest_path_.poses[nearest + 2].pose.position;
  const double seg_yaw = std::atan2(b.y - a.y, b.x - a.x);
  double diff = seg_yaw - yaw;
  while (diff >  M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  constexpr double TURN_THRESH = M_PI / 4.0;
  if (diff >  TURN_THRESH) return 'L';
  if (diff < -TURN_THRESH) return 'R';
  return std::nullopt;
}

FsmInputs BehaviorPlannerNode::build_inputs_(double now_s)
{
  std::lock_guard<std::mutex> lk(cache_mtx_);
  FsmInputs in;

  const auto t_now = this->now();
  const bool state_fresh =
    have_state_ && (t_now - last_state_time_).seconds() < stale_state_s_;
  const bool lane_fresh =
    last_lane_time_.nanoseconds() != 0 &&
    (t_now - last_lane_time_).seconds() < stale_lane_s_;
  const bool objects_fresh =
    last_objects_time_.nanoseconds() != 0 &&
    (t_now - last_objects_time_).seconds() < stale_objects_s_;
  const bool freespace_fresh =
    last_freespace_time_.nanoseconds() != 0 &&
    (t_now - last_freespace_time_).seconds() < stale_freespace_s_;

  in.state_valid      = state_fresh;
  in.speed            = state_fresh ? latest_state_.speed : 0.0;
  in.state_confidence = state_fresh ? latest_state_.confidence : 0.0;
  in.imu_confidence   = last_imu_conf_;
  in.emergency        = emergency_flag_;
  in.lane_valid       = lane_fresh;
  in.lane_dashed      = lane_fresh && latest_lane_.confidence > lane_dashed_thresh_;

  // ----- DetectionTimer pass: aggregate per-class closest distance -------
  // We collapse `classes_*` lists down to canonical keys ("pedestrian",
  // "vehicle", …) which DetectionTimer is indexed by. This lets the
  // detector ship multiple class strings per category (e.g. car + truck)
  // without changing FSM logic.
  std::unordered_map<std::string, double> closest;
  if (objects_fresh) {
    auto consider = [&](const bfmc_msgs::msg::DetectedObject & o,
                        const std::vector<std::string> & matches,
                        const std::string & canonical)
    {
      if (!class_matches(o.class_name, matches)) return;
      const double d = euclid_to_obj(o);
      auto it = closest.find(canonical);
      if (it == closest.end() || d < it->second) closest[canonical] = d;
    };
    for (const auto & o : latest_objects_.objects) {
      consider(o, classes_pedestrian_,    "pedestrian");
      consider(o, classes_vehicle_,       "vehicle");
      consider(o, classes_traffic_light_, "trafficlight");
      consider(o, classes_stop_sign_,     "stop");
      consider(o, classes_crosswalk_,     "crosswalk");
      consider(o, classes_roadblock_,     "roadblock");
      consider(o, classes_parking_,       "parking");
      consider(o, classes_roundabout_,    "roundabout");
      consider(o, classes_highway_entry_, "highway_entry");
      consider(o, classes_highway_exit_,  "highway_exit");
      consider(o, classes_no_entry_,      "no_entry");
      consider(o, classes_priority_,      "priority");
      consider(o, classes_onewayroad_,    "onewayroad");
    }
  }
  for (auto & [k, dt] : det_timers_) {
    auto it = closest.find(k);
    if (it != closest.end()) dt.update(true,  it->second,                        now_s);
    else                     dt.update(false, std::numeric_limits<double>::infinity(), now_s);
  }

  // ----- Map confirmed timers → FsmInputs --------------------------------
  in.pedestrian_ahead = det_timers_.at("pedestrian").confirmed();
  in.pedestrian_dist  = det_timers_.at("pedestrian").closest_distance();

  in.vehicle_ahead    = det_timers_.at("vehicle").confirmed();
  in.vehicle_dist     = det_timers_.at("vehicle").closest_distance();

  // Visual trafficlight class: treat as "potentially red" trigger until V2X
  // confirms otherwise. If we have ANY semaphore on /v2x/semaphores marked
  // green or yellow, suppress the visual stop (the V2X is authoritative —
  // BFMC scoring relies on it).
  in.traffic_light_red_ahead = det_timers_.at("trafficlight").confirmed() &&
                               det_timers_.at("trafficlight").closest_distance()
                                 <= tuning_.traffic_light_dist_thresh;
  for (const auto & [_id, st] : semaphore_state_) {
    if (st == "green" || st == "yellow") {
      in.traffic_light_red_ahead = false;   // V2X overrides — proceed
      break;
    }
  }

  in.stop_sign_ahead = det_timers_.at("stop").confirmed() &&
                       det_timers_.at("stop").closest_distance()
                         <= tuning_.stop_sign_dist_thresh;
  in.stop_sign_dist  = det_timers_.at("stop").closest_distance();

  in.crosswalk_ahead = det_timers_.at("crosswalk").confirmed() &&
                       det_timers_.at("crosswalk").closest_distance()
                         <= tuning_.crosswalk_dist_thresh;

  in.roadblock_ahead = det_timers_.at("roadblock").confirmed() &&
                       det_timers_.at("roadblock").closest_distance()
                         <= tuning_.roadblock_dist_thresh;

  in.highway_entering = det_timers_.at("highway_entry").confirmed();
  in.highway_exiting  = det_timers_.at("highway_exit").confirmed();

  // ----- Race-day BFMC sign set (no_entry / priority / onewayroad) ------
  in.no_entry_ahead = det_timers_.at("no_entry").confirmed() &&
                      det_timers_.at("no_entry").closest_distance()
                        <= tuning_.no_entry_dist_thresh;
  in.no_entry_dist  = det_timers_.at("no_entry").closest_distance();
  // `priority` and `onewayroad` are modifiers — confirmed flag only.
  in.has_priority   = det_timers_.at("priority").confirmed();
  in.on_oneway      = det_timers_.at("onewayroad").confirmed();

  // Parking / roundabout zones: ANY of (a) confirmed-class sign AT CLOSE
  // RANGE, OR (b) nav_flag from global planner (zones.yaml side-car).
  // Race-day fix: distance gate added — without it a single ghost sign far
  // away (e.g. wall texture in a static bench scene) locked PARK forever.
  const bool ros_parking    = det_timers_.at("parking").confirmed() &&
                              det_timers_.at("parking").closest_distance()
                                <= tuning_.parking_dist_thresh;
  const bool ros_roundabout = det_timers_.at("roundabout").confirmed() &&
                              det_timers_.at("roundabout").closest_distance()
                                <= tuning_.roundabout_dist_thresh;
  in.parking_zone    = ros_parking    || nav_flag_ == "parking";
  in.roundabout_zone = ros_roundabout || nav_flag_ == "roundabout";

  // on_highway is purely nav_flag-driven (entry/exit are visual signs).
  in.on_highway = (nav_flag_ == "highway");

  // ----- FreeSpace overtake check ---------------------------------------
  in.freespace_left_clear = freespace_fresh &&
    freespace_left_clear_(latest_freespace_,
                          freespace_left_col_start_,
                          freespace_left_col_end_,
                          freespace_clear_thresh_);

  // ----- Intersection turn from path lookahead ---------------------------
  in.intersection_turn = derive_intersection_turn_();

  return in;
}

void BehaviorPlannerNode::fsm_tick()
{
  if (!active_) return;
  const double now_s = this->now().seconds();
  const auto in   = build_inputs_(now_s);
  const Maneuver out = fsm_.step(in, tuning_, now_s);
  double target_speed = fsm_.recommended_speed(tuning_);

  // Mission-supervisor speed override. ONLY applies in LANE_KEEP / FOLLOW
  // (overtake-intent) — these are the cruise regimes where the YAML profile
  // (city 0.45, highway 0.70, etc.) should win over the FSM's safer
  // hardcoded defaults. Every safety-priority state keeps the FSM speed:
  //   PEDESTRIAN_WAIT/STOP/EMERGENCY_STOP keep v=0
  //   CROSSWALK_SLOW keeps the slow crosswalk speed (mission YAML 0.20 is
  //     slower than fsm 0.10, so we don't want the YAML to RAISE the
  //     crosswalk speed — keep FSM)
  //   PARK / ROUNDABOUT_* / HIGHWAY_ENTER/EXIT (ramp) / INTERSECTION_* all
  //     keep their FSM-computed maneuver speed.
  // The "raise" direction is the only one that matters here — the FSM may
  // already be slower than the YAML for safety, and we honor that.
  if (use_mission_speed_) {
    const double mission_speed = mission_speed_limit_.load(std::memory_order_relaxed);
    if (mission_speed > 0.0 &&
        (out == Maneuver::LANE_KEEP || out == Maneuver::FOLLOW))
    {
      // For FOLLOW (vehicle-ahead overtake-intent): cap by the FSM's tail-decel
      //   so we never accelerate INTO a leader; use min(fsm, mission).
      // For LANE_KEEP: use max — the YAML is the cruise target.
      double new_speed = (out == Maneuver::FOLLOW)
          ? std::min(target_speed, mission_speed)
          : std::max(target_speed, mission_speed);
      target_speed = new_speed;
    }
  }

  std_msgs::msg::String bs;  bs.data = maneuver_to_str(out);
  pub_behavior_->publish(bs);
  std_msgs::msg::Float32 ts; ts.data = static_cast<float>(target_speed);
  pub_target_speed_->publish(ts);

  // Race-day overtake: lateral offset (m) for the local planner. 0 when
  // not overtaking; non-zero only during OVERTAKE sub-FSM phases.
  std_msgs::msg::Float32 lo;
  lo.data = static_cast<float>(fsm_.lateral_offset());
  pub_lateral_offset_->publish(lo);

  // Debug
  std_msgs::msg::String cs; cs.data = maneuver_to_str(out);
  pub_dbg_state_->publish(cs);
  std_msgs::msg::String nm; nm.data = maneuver_to_str(out);
  pub_dbg_next_->publish(nm);
  if (fsm_.just_transitioned()) {
    std_msgs::msg::String tr;
    tr.data = std::string(maneuver_to_str(fsm_.previous())) + " -> " +
              std::string(maneuver_to_str(out));
    pub_dbg_transition_->publish(tr);
    RCLCPP_INFO(get_logger(), "FSM: %s  (target=%.2f m/s)", tr.data.c_str(), target_speed);
  }

  // CSV row
  if (enable_csv_logging_ && csv_fsm_.is_open()) {
    csv_fsm_ << std::fixed << std::setprecision(6) << now_s << ","
             << maneuver_to_str(out) << ","
             << std::setprecision(3) << target_speed << ","
             << in.speed << ","
             << in.imu_confidence << ","
             << in.state_confidence << ","
             << (in.pedestrian_ahead     ? 1 : 0) << ","
             << (in.vehicle_ahead        ? 1 : 0) << ","
             << (in.stop_sign_ahead      ? 1 : 0) << ","
             << (in.traffic_light_red_ahead ? 1 : 0) << ","
             << (in.crosswalk_ahead      ? 1 : 0) << ","
             << (in.on_highway           ? 1 : 0)
             << "\n";
    if (++csv_tick_counter_ >= CSV_FLUSH_EVERY) {
      csv_fsm_.flush();
      csv_tick_counter_ = 0;
    }
  }
}

/* =========================== CSV log ============================ */
bool BehaviorPlannerNode::open_csv_logs_()
{
  std::error_code ec;
  std::filesystem::create_directories(csv_log_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(get_logger(),
      "cannot create CSV dir '%s': %s", csv_log_dir_.c_str(), ec.message().c_str());
    return false;
  }
  const std::string path =
    (std::filesystem::path(csv_log_dir_) / "fsm.csv").string();
  csv_fsm_.open(path, std::ios::out | std::ios::app | std::ios::ate);
  if (!csv_fsm_.is_open()) return false;
  if (csv_fsm_.tellp() == std::streampos(0)) {
    csv_fsm_ << "timestamp,maneuver,target_speed,speed,imu_conf,state_conf,"
             << "ped,veh,stop,red,crosswalk,highway\n";
  }
  return true;
}

void BehaviorPlannerNode::close_csv_logs_()
{
  if (csv_fsm_.is_open()) { csv_fsm_.flush(); csv_fsm_.close(); }
}

}  // namespace behavior_planner_cpp
