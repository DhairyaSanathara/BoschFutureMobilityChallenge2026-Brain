#include "object_tracker_cpp/object_tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "bfmc_msgs/msg/detected_object.hpp"

namespace object_tracker_cpp
{

ObjectTrackerNode::ObjectTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("object_tracker_node", options),
  last_log_time_(this->now())
{
  // -------------------- Parameters ----------------------------------------
  ByteTrackerParams p;
  p.conf_high       = declare_parameter<double>("conf_high",       p.conf_high);
  p.conf_low        = declare_parameter<double>("conf_low",        p.conf_low);
  p.max_assoc_dist  = declare_parameter<double>("max_assoc_dist",  p.max_assoc_dist);
  p.n_init          = static_cast<std::uint64_t>(
    declare_parameter<int>("n_init",
      static_cast<int>(p.n_init)));
  p.max_age         = static_cast<std::uint64_t>(
    declare_parameter<int>("max_age",
      static_cast<int>(p.max_age)));
  p.max_lost_frames = static_cast<std::uint64_t>(
    declare_parameter<int>("max_lost_frames",
      static_cast<int>(p.max_lost_frames)));

  input_topic_  = declare_parameter<std::string>("input_topic",  input_topic_);
  output_topic_ = declare_parameter<std::string>("output_topic", output_topic_);

  tracker_ = std::make_unique<ByteTracker>(p);

  // -------------------- ROS interfaces ------------------------------------
  // Detector publishes ObjectArray Reliably (small message), so we mirror.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  sub_ = create_subscription<bfmc_msgs::msg::ObjectArray>(
    input_topic_, qos,
    std::bind(&ObjectTrackerNode::on_objects, this, std::placeholders::_1));
  pub_ = create_publisher<bfmc_msgs::msg::ObjectArray>(output_topic_, qos);

  // -------------------- Debug topics (Phase 3.1) --------------------------
  // BEST_EFFORT, lightweight, always on. Aggregate cost at 30 Hz with
  // < 20 live tracks is < 1 KB/s — small enough to leave running on the
  // track for race-day diagnostics without a gating param.
  rclcpp::QoS dbg_qos{rclcpp::KeepLast(5)};
  dbg_qos.best_effort();
  debug_track_count_pub_   = create_publisher<std_msgs::msg::UInt32>(
                               "/perception/debug/track_count",         dbg_qos);
  debug_active_tracks_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
                               "/perception/debug/active_tracks",       dbg_qos);
  debug_latency_pub_       = create_publisher<std_msgs::msg::Float64>(
                               "/perception/debug/tracker_latency_ms",  dbg_qos);
  debug_id_switch_pub_     = create_publisher<std_msgs::msg::UInt32>(
                               "/perception/debug/id_switch_count",     dbg_qos);

  // Preallocate the output buffer — bounded by uint16 track-id range but
  // realistically capped by detector output (≤ 64 objects/frame).
  out_msg_.objects.reserve(64);

  RCLCPP_INFO(get_logger(),
    "object_tracker: in='%s' out='%s' conf_high=%.2f conf_low=%.2f "
    "max_assoc_dist=%.2f n_init=%lu max_age=%lu max_lost=%lu "
    "dt_clamp=[%.3f,%.3f]s",
    input_topic_.c_str(), output_topic_.c_str(),
    p.conf_high, p.conf_low, p.max_assoc_dist,
    static_cast<unsigned long>(p.n_init),
    static_cast<unsigned long>(p.max_age),
    static_cast<unsigned long>(p.max_lost_frames),
    DT_MIN, DT_MAX);
}

void ObjectTrackerNode::on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();
  ++frame_index_;

  // ------ Real-timestamp dt (Phase 3.1) -----------------------------------
  // dt comes from upstream ObjectArray.stamp, which is set by the detector
  // to the source-image acquisition time. We deliberately do NOT use any
  // wall clock or fixed 1/30 — frame drops in the detector pipeline are
  // common and prediction must remain stable across them.
  const rclcpp::Time stamp(msg->stamp);
  double dt = 0.0;
  bool   dt_valid = false;
  if (have_prev_stamp_) {
    const double raw = (stamp - last_msg_stamp_).seconds();
    if (raw <= 0.0) {
      // Clock jump backwards, replay, or identical-stamp duplicate — restart
      // the dt chain.
      have_prev_stamp_ = false;
    } else if (raw > 1.0) {
      // Big gap (>1 s) ⇒ detector restart or long stall. Don't propagate a
      // huge prediction step; restart the chain so the next frame establishes
      // a fresh dt baseline.
      have_prev_stamp_ = false;
    } else {
      dt = std::clamp(raw, DT_MIN, DT_MAX);
      dt_valid = true;
    }
  }
  if (!have_prev_stamp_) {
    have_prev_stamp_ = true;
  }
  last_msg_stamp_ = stamp;
  (void) dt_valid;     // dt==0 on first frame is the right thing (no predict step)

  // ------ Re-shape DetectedObject[] → Detection[] for the tracker ---------
  std::vector<Detection> dets;
  dets.reserve(msg->objects.size());
  for (const auto & o : msg->objects) {
    Detection d;
    d.x          = o.position.x;
    d.y          = o.position.y;
    d.confidence = o.confidence;
    d.class_id   = o.id;            // detector packs class_id into `id`
    d.class_name = o.class_name;
    dets.push_back(std::move(d));
  }

  const auto live = tracker_->step(dt, std::move(dets));

  // ------ ID-switch heuristic update --------------------------------------
  // Done AFTER step() so we see this step's births and deaths.
  update_id_switch_count_(frame_index_);

  // ------ Publish: SAME message type. Phase 3.1 keeps DetectedObject.id ---
  //        as the detector class id (semantic preserved!) and writes the
  //        stable tracker identity into the new DetectedObject.track_id
  //        field that bfmc_msgs grew this phase.
  out_msg_.stamp = msg->stamp;       // preserve detection timestamp
  out_msg_.objects.clear();
  for (const auto * t : live) {
    bfmc_msgs::msg::DetectedObject o;
    o.id            = t->class_id();        // class id — UNCHANGED semantic
    o.class_name    = t->class_name();
    o.position.x    = t->x();
    o.position.y    = t->y();
    o.position.z    = 0.0;
    o.velocity.x    = t->vx();              // KF-smoothed — was 0 from detector
    o.velocity.y    = t->vy();
    o.velocity.z    = 0.0;
    o.confidence    = t->confidence();
    o.track_id      = t->track_id();        // NEW field, stable across frames
    out_msg_.objects.push_back(std::move(o));
  }
  pub_->publish(out_msg_);

  // ------ Debug topics ----------------------------------------------------
  const auto t_end = std::chrono::steady_clock::now();
  const double tick_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  {
    std_msgs::msg::UInt32 m;
    m.data = static_cast<std::uint32_t>(live.size());
    debug_track_count_pub_->publish(m);
  }
  {
    std_msgs::msg::UInt16MultiArray m;
    m.data.reserve(live.size());
    for (const auto * t : live) m.data.push_back(t->track_id());
    debug_active_tracks_pub_->publish(m);
  }
  {
    std_msgs::msg::Float64 m;
    m.data = tick_ms;
    debug_latency_pub_->publish(m);
  }
  {
    std_msgs::msg::UInt32 m;
    m.data = id_switch_count_;
    debug_id_switch_pub_->publish(m);
  }

  // ------ Periodic INFO log -----------------------------------------------
  ++msgs_in_window_;
  const auto now_ros = this->now();
  if ((now_ros - last_log_time_).seconds() >= 1.0) {
    RCLCPP_INFO(get_logger(),
      "object_tracker: %lu msg/s, %zu live tracks, %lu total ever, "
      "id_switches=%u, last tick %.2f ms",
      static_cast<unsigned long>(msgs_in_window_),
      live.size(),
      static_cast<unsigned long>(tracker_->total_tracks_created()),
      id_switch_count_,
      tick_ms);
    msgs_in_window_ = 0;
    last_log_time_  = now_ros;
  }
}

void ObjectTrackerNode::update_id_switch_count_(std::uint64_t now_frame)
{
  // Drop dead-records older than the window.
  while (!recent_deaths_.empty() &&
         now_frame - recent_deaths_.front().frame_index > ID_SWITCH_WINDOW)
  {
    recent_deaths_.pop_front();
  }

  // For every birth this step: if a matching death is in the buffer
  // (same class, within ID_SWITCH_RADIUS), count one switch and consume it
  // so a single death cannot be charged against multiple births.
  for (const auto & birth : tracker_->last_births()) {
    auto it = recent_deaths_.begin();
    while (it != recent_deaths_.end()) {
      if (it->class_id != birth.class_id) { ++it; continue; }
      const double dx = it->x - birth.x;
      const double dy = it->y - birth.y;
      if (dx*dx + dy*dy <= ID_SWITCH_RADIUS * ID_SWITCH_RADIUS) {
        ++id_switch_count_;
        it = recent_deaths_.erase(it);
        break;       // one death → one switch → next birth
      } else {
        ++it;
      }
    }
  }

  // Append this step's deaths to the rolling buffer.
  for (const auto & d : tracker_->last_deaths()) {
    recent_deaths_.push_back({d.track_id, d.class_id, d.x, d.y, now_frame});
  }
}

}  // namespace object_tracker_cpp
