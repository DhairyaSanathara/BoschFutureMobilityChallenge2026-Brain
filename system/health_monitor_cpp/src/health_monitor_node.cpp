#include "health_monitor_cpp/health_monitor_node.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace bfmc_system
{

namespace
{

// Lower-jitter wall-clock seconds. Used uniformly so rate windows trim against
// the same time source the arrivals were stamped with.
double wall_seconds(const rclcpp::Node & n)
{
  return n.now().seconds();
}

// Best-effort, SensorDataQoS-style QoS for high-rate sensor streams.
// Braces (not parens) avoid the most-vexing-parse — `QoS q(KeepLast(d));`
// is otherwise read as a function declaration by the compiler.
rclcpp::QoS sensor_qos(std::size_t depth = 5)
{
  rclcpp::QoS q{rclcpp::KeepLast(depth)};
  q.best_effort();
  return q;
}

// Reliable QoS for state-like topics.
rclcpp::QoS reliable_qos(std::size_t depth = 10)
{
  rclcpp::QoS q{rclcpp::KeepLast(depth)};
  q.reliable();
  return q;
}

}  // namespace

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("health_monitor_node", options),
  launch_time_(this->now())
{
  // ------ parameters --------------------------------------------------------
  rate_window_s_         = declare_parameter<double>("rate_window_s",        rate_window_s_);
  metrics_period_s_      = declare_parameter<double>("metrics_period_s",     metrics_period_s_);
  summary_period_s_      = declare_parameter<double>("summary_period_s",     summary_period_s_);
  validation_output_dir_ = declare_parameter<std::string>(
      "validation_output_dir",
      std::string(""));   // empty default ⇒ no file written

  // Per-topic stale thresholds. Defaults sized to ~5× publish period so a
  // single dropped frame does not flap stale_count.
  stats_state_      .stale_threshold_s = declare_parameter<double>("stale_state_s",      0.05);   //  50 ms  (EKF ~100 Hz)
  stats_tf_         .stale_threshold_s = declare_parameter<double>("stale_tf_s",         0.05);   //  50 ms  (TF tracks state)
  stats_image_      .stale_threshold_s = declare_parameter<double>("stale_camera_s",     0.20);   // 200 ms  (RGB ~30 Hz)
  stats_lane_geom_  .stale_threshold_s = declare_parameter<double>("stale_lane_s",       0.20);   // 200 ms  (lane ~20 Hz)
  stats_imu_        .stale_threshold_s = declare_parameter<double>("stale_imu_s",        0.05);   //  50 ms  (IMU ~100 Hz)
  stats_correction_ .stale_threshold_s = declare_parameter<double>("stale_correction_s", 0.20);   // 200 ms  (drift ~20 Hz)
  stats_lane_heading_.stale_threshold_s = declare_parameter<double>("stale_lane_heading_s", 0.20);
  stats_tracked_    .stale_threshold_s = declare_parameter<double>("stale_tracked_s",    0.20);   // 200 ms  (tracker ~30 Hz, mirrors detector cadence)

  // ------ resolve run directory ---------------------------------------------
  // If validation_output_dir is set we materialise <dir>/run-<ts>/ once at
  // startup. validation_summary.yaml is overwritten in place on each
  // summary_tick(); the file is therefore always a current snapshot.
  if (!validation_output_dir_.empty()) {
    try {
      std::filesystem::create_directories(validation_output_dir_);
      run_dir_ = (std::filesystem::path(validation_output_dir_) /
                  make_timestamp_dir_name(launch_time_)).string();
      std::filesystem::create_directories(run_dir_);
      RCLCPP_INFO(get_logger(), "health_monitor: run directory = %s", run_dir_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "health_monitor: cannot create run directory '%s': %s",
        validation_output_dir_.c_str(), e.what());
      run_dir_.clear();
    }
  } else {
    RCLCPP_INFO(get_logger(),
      "health_monitor: validation_output_dir empty — no YAML summary will be written");
  }

  // ------ subscriptions -----------------------------------------------------
  sub_state_ = create_subscription<bfmc_msgs::msg::VehicleState>(
      "/vehicle/state", reliable_qos(),
      std::bind(&HealthMonitorNode::on_state, this, std::placeholders::_1));

  sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", reliable_qos(100),
      std::bind(&HealthMonitorNode::on_tf, this, std::placeholders::_1));

  sub_correction_ = create_subscription<std_msgs::msg::Float64>(
      "/fusion/yaw_bias_correction", reliable_qos(),
      std::bind(&HealthMonitorNode::on_correction, this, std::placeholders::_1));

  // /fusion/debug/lane_heading only exists when drift_estimator's
  // enable_debug_topics is true. Subscribing when absent is harmless — the
  // rate will simply read 0 Hz and no values will accumulate.
  sub_lane_heading_ = create_subscription<std_msgs::msg::Float64>(
      "/fusion/debug/lane_heading", sensor_qos(),
      std::bind(&HealthMonitorNode::on_lane_heading, this, std::placeholders::_1));

  sub_lane_geom_ = create_subscription<bfmc_msgs::msg::LaneGeometry>(
      "/perception/lane_geometry", sensor_qos(),
      std::bind(&HealthMonitorNode::on_lane_geom, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/microros/imu_raw", sensor_qos(),
      std::bind(&HealthMonitorNode::on_imu, this, std::placeholders::_1));

  // Camera RGB at 30 Hz, 1280×720×3 = ~2.7 MB per frame. We use SensorDataQoS
  // depth 1 + best-effort so DDS drops on backpressure — we never block the
  // producer, and the only cost we pay is one deserialise + header read per
  // delivered frame. We do NOT touch msg->data.
  rclcpp::QoS image_qos{rclcpp::KeepLast(1)};
  image_qos.best_effort();
  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", image_qos,
      std::bind(&HealthMonitorNode::on_image, this, std::placeholders::_1));

  // Phase 3.1: tracker output. Reliable mirrors object_detection_cpp /
  // object_tracker_cpp's publish QoS so we don't drop frames at the monitor.
  sub_tracked_ = create_subscription<bfmc_msgs::msg::ObjectArray>(
      "/perception/tracked_objects", reliable_qos(),
      std::bind(&HealthMonitorNode::on_tracked, this, std::placeholders::_1));

  // ------ publishers --------------------------------------------------------
  status_pub_  = create_publisher<std_msgs::msg::String>("/system/health/status",
                                                          reliable_qos(5));
  metrics_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
                   "/system/health/metrics", reliable_qos(5));

  // ------ timers ------------------------------------------------------------
  metrics_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(metrics_period_s_ * 1000.0)),
      std::bind(&HealthMonitorNode::metrics_tick, this));

  summary_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(summary_period_s_ * 1000.0)),
      std::bind(&HealthMonitorNode::summary_tick, this));

  RCLCPP_INFO(get_logger(),
    "health_monitor: window=%.1fs metrics=%.1fs summary=%.1fs",
    rate_window_s_, metrics_period_s_, summary_period_s_);
}

HealthMonitorNode::~HealthMonitorNode()
{
  // Last-chance write of the validation summary so a clean shutdown captures
  // the final state of the run.
  if (!run_dir_.empty()) write_summary_yaml();
}

// ---------- callbacks -------------------------------------------------------

void HealthMonitorNode::register_arrival(TopicStats & ts, double stamp_sec_or_zero)
{
  const double now_s = wall_seconds(*this);
  ts.arrival_times.push_back(now_s);
  ts.last_arrival_sec = now_s;
  ++ts.total;
  if (stamp_sec_or_zero > 0.0) {
    const double latency = now_s - stamp_sec_or_zero;
    if (std::isfinite(latency) && latency >= 0.0 && latency < 5.0) {
      ts.latency.add(latency);
    }
  }
  ts.trim(now_s, rate_window_s_);
}

void HealthMonitorNode::on_state(const bfmc_msgs::msg::VehicleState::SharedPtr /*msg*/)
{
  // VehicleState has no header.stamp — count only.
  register_arrival(stats_state_, 0.0);
}

void HealthMonitorNode::on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  double stamp = 0.0;
  if (!msg->transforms.empty()) {
    const auto & h = msg->transforms.front().header.stamp;
    stamp = static_cast<double>(h.sec) + 1e-9 * static_cast<double>(h.nanosec);
  }
  register_arrival(stats_tf_, stamp);
}

void HealthMonitorNode::on_correction(const std_msgs::msg::Float64::SharedPtr msg)
{
  register_arrival(stats_correction_, 0.0);
  yaw_bias_stats_.add(msg->data);
}

void HealthMonitorNode::on_lane_heading(const std_msgs::msg::Float64::SharedPtr msg)
{
  register_arrival(stats_lane_heading_, 0.0);
  lane_heading_stats_.add(msg->data);
}

void HealthMonitorNode::on_lane_geom(const bfmc_msgs::msg::LaneGeometry::SharedPtr /*msg*/)
{
  // LaneGeometry has no header.stamp — count only.
  register_arrival(stats_lane_geom_, 0.0);
}

void HealthMonitorNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & h = msg->header.stamp;
  const double stamp = static_cast<double>(h.sec) + 1e-9 * static_cast<double>(h.nanosec);
  register_arrival(stats_imu_, stamp);
}

void HealthMonitorNode::on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const auto & h = msg->header.stamp;
  const double stamp = static_cast<double>(h.sec) + 1e-9 * static_cast<double>(h.nanosec);
  register_arrival(stats_image_, stamp);
}

void HealthMonitorNode::on_tracked(const bfmc_msgs::msg::ObjectArray::SharedPtr msg)
{
  // ObjectArray.stamp is the detector frame time. Use it for latency stats
  // so the monitor's tracked-latency measurement captures the full
  // detector→tracker→DDS pipeline.
  const auto & h = msg->stamp;
  const double stamp = static_cast<double>(h.sec) + 1e-9 * static_cast<double>(h.nanosec);
  register_arrival(stats_tracked_, stamp);
}

// ---------- staleness -------------------------------------------------------

void HealthMonitorNode::detect_stale(TopicStats & ts)
{
  // If we have *never* heard from a topic, do not increment stale_count — it
  // would otherwise inflate every tick before the producer starts. Producers
  // that genuinely never come up will be visible from rate == 0.
  if (ts.total == 0) return;
  const double age = wall_seconds(*this) - ts.last_arrival_sec;
  if (age > ts.stale_threshold_s) {
    ++ts.stale_count;
    ++warning_count_;
  }
}

// ---------- metrics tick ----------------------------------------------------

void HealthMonitorNode::metrics_tick()
{
  const double now_s = wall_seconds(*this);

  // Trim each rolling window so a paused producer drops out of the rate calc.
  stats_state_       .trim(now_s, rate_window_s_);
  stats_tf_          .trim(now_s, rate_window_s_);
  stats_correction_  .trim(now_s, rate_window_s_);
  stats_lane_heading_.trim(now_s, rate_window_s_);
  stats_lane_geom_   .trim(now_s, rate_window_s_);
  stats_imu_         .trim(now_s, rate_window_s_);
  stats_image_       .trim(now_s, rate_window_s_);
  stats_tracked_     .trim(now_s, rate_window_s_);

  // Tick staleness counters (one detection per metrics tick — typically 1 Hz).
  detect_stale(stats_state_);
  detect_stale(stats_tf_);
  detect_stale(stats_image_);
  detect_stale(stats_lane_geom_);
  detect_stale(stats_imu_);
  detect_stale(stats_tracked_);    // Phase 3.1: tracker stall ⇒ +warning_count_

  // ---- structured metrics --------------------------------------------------
  std_msgs::msg::Float32MultiArray m;
  m.data.resize(METRICS_LAYOUT_SIZE, 0.0f);

  m.data[0]  = static_cast<float>(stats_state_      .rate(rate_window_s_));
  m.data[1]  = static_cast<float>(stats_tf_         .rate(rate_window_s_));
  m.data[2]  = static_cast<float>(stats_image_      .rate(rate_window_s_));
  m.data[3]  = static_cast<float>(stats_lane_geom_  .rate(rate_window_s_));
  m.data[4]  = static_cast<float>(stats_imu_        .rate(rate_window_s_));
  m.data[5]  = static_cast<float>(stats_correction_ .rate(rate_window_s_));
  m.data[6]  = static_cast<float>(yaw_bias_stats_    .mean);
  m.data[7]  = static_cast<float>(yaw_bias_stats_    .stddev());
  m.data[8]  = static_cast<float>(lane_heading_stats_.mean);
  m.data[9]  = static_cast<float>(lane_heading_stats_.stddev());
  m.data[10] = static_cast<float>(stats_state_      .stale_count);
  m.data[11] = static_cast<float>(stats_tf_         .stale_count);
  m.data[12] = static_cast<float>(stats_image_      .stale_count);
  m.data[13] = static_cast<float>(stats_lane_geom_  .stale_count);
  m.data[14] = static_cast<float>(stats_imu_        .stale_count);
  m.data[15] = static_cast<float>(stats_imu_.latency.max_val * 1000.0);
  m.data[16] = static_cast<float>(warning_count_);
  m.data[17] = static_cast<float>(now_s - launch_time_.seconds());
  m.data[18] = static_cast<float>(stats_tracked_.rate(rate_window_s_));    // Phase 3.1
  m.data[19] = static_cast<float>(stats_tracked_.stale_count);             // Phase 3.1

  metrics_pub_->publish(m);

  // ---- human-readable status ----------------------------------------------
  std::ostringstream os;
  os << std::fixed << std::setprecision(1);
  os << "[health] uptime=" << (now_s - launch_time_.seconds()) << "s "
     << "rates(Hz): state=" << m.data[0]
     << " tf=" << m.data[1]
     << " cam=" << m.data[2]
     << " lane=" << m.data[3]
     << " imu=" << m.data[4]
     << " correction=" << m.data[5]
     << " tracked=" << m.data[18]
     << " | yaw_bias mean=" << std::setprecision(5) << m.data[6]
     << " std=" << m.data[7]
     << std::setprecision(1)
     << " | stale[state/tf/cam/lane/imu/tracked]="
     << static_cast<int>(m.data[10]) << "/"
     << static_cast<int>(m.data[11]) << "/"
     << static_cast<int>(m.data[12]) << "/"
     << static_cast<int>(m.data[13]) << "/"
     << static_cast<int>(m.data[14]) << "/"
     << static_cast<int>(m.data[19])
     << " warn=" << warning_count_;

  std_msgs::msg::String s;
  s.data = os.str();
  status_pub_->publish(s);
}

// ---------- summary YAML ----------------------------------------------------

void HealthMonitorNode::summary_tick()
{
  if (run_dir_.empty()) return;
  if (!write_summary_yaml()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "health_monitor: failed to write validation_summary.yaml");
  }
}

bool HealthMonitorNode::write_summary_yaml() const
{
  if (run_dir_.empty()) return false;
  const std::string path = (std::filesystem::path(run_dir_) / "validation_summary.yaml").string();

  std::ofstream out(path, std::ios::trunc);
  if (!out) return false;

  const double now_s = const_cast<HealthMonitorNode *>(this)->now().seconds();
  const double uptime = now_s - launch_time_.seconds();

  out << std::fixed << std::setprecision(3);
  out << "# bfmc_autonomy validation summary — overwritten every "
      << summary_period_s_ << "s\n";
  out << "run_dir: " << run_dir_ << "\n";
  out << "uptime_s: " << uptime << "\n";
  out << "rate_window_s: " << rate_window_s_ << "\n\n";

  out << "rates_hz:\n";
  out << "  vehicle_state: " << stats_state_      .rate(rate_window_s_) << "\n";
  out << "  tf: "            << stats_tf_         .rate(rate_window_s_) << "\n";
  out << "  camera: "        << stats_image_      .rate(rate_window_s_) << "\n";
  out << "  lane: "          << stats_lane_geom_  .rate(rate_window_s_) << "\n";
  out << "  imu: "           << stats_imu_        .rate(rate_window_s_) << "\n";
  out << "  correction: "    << stats_correction_ .rate(rate_window_s_) << "\n";
  out << "  lane_heading: "  << stats_lane_heading_.rate(rate_window_s_) << "\n";
  out << "  tracked_objects: " << stats_tracked_   .rate(rate_window_s_) << "\n\n";

  out << std::setprecision(6);
  out << "yaw_bias:\n";
  out << "  n: "    << yaw_bias_stats_.n      << "\n";
  out << "  mean: " << yaw_bias_stats_.mean   << "\n";
  out << "  std: "  << yaw_bias_stats_.stddev() << "\n\n";

  out << "lane_heading:\n";
  out << "  n: "    << lane_heading_stats_.n      << "\n";
  out << "  mean: " << lane_heading_stats_.mean   << "\n";
  out << "  std: "  << lane_heading_stats_.stddev() << "\n\n";

  out << std::setprecision(3);
  out << "stale_message_count:\n";
  out << "  vehicle_state: " << stats_state_   .stale_count << "\n";
  out << "  tf: "            << stats_tf_      .stale_count << "\n";
  out << "  camera: "        << stats_image_   .stale_count << "\n";
  out << "  lane: "          << stats_lane_geom_.stale_count << "\n";
  out << "  imu: "           << stats_imu_    .stale_count << "\n";
  out << "  tracked_objects: " << stats_tracked_.stale_count << "\n\n";

  out << "dropped_frames:\n";
  out << "  # 'dropped' is inferred from rate shortfall vs. expected publish\n";
  out << "  # rate (vehicle_state ~100, tf ~100, camera ~30, lane ~20, imu ~100).\n";
  out << "  # Negative ⇒ ran faster than expected; positive ⇒ shortfall.\n";
  out << "  vehicle_state: " << std::max(0.0, 100.0 - stats_state_  .rate(rate_window_s_)) * uptime << "\n";
  out << "  tf: "            << std::max(0.0, 100.0 - stats_tf_     .rate(rate_window_s_)) * uptime << "\n";
  out << "  camera: "        << std::max(0.0,  30.0 - stats_image_  .rate(rate_window_s_)) * uptime << "\n";
  out << "  lane: "          << std::max(0.0,  20.0 - stats_lane_geom_.rate(rate_window_s_)) * uptime << "\n";
  out << "  imu: "           << std::max(0.0, 100.0 - stats_imu_    .rate(rate_window_s_)) * uptime << "\n\n";

  out << "latency_ms:\n";
  out << "  imu_avg: "    << stats_imu_  .latency.mean    * 1000.0 << "\n";
  out << "  imu_max: "    << stats_imu_  .latency.max_val * 1000.0 << "\n";
  out << "  imu_std: "    << stats_imu_  .latency.stddev() * 1000.0 << "\n";
  out << "  camera_avg: " << stats_image_.latency.mean    * 1000.0 << "\n";
  out << "  camera_max: " << stats_image_.latency.max_val * 1000.0 << "\n";
  out << "  camera_std: " << stats_image_.latency.stddev() * 1000.0 << "\n";
  out << "  tf_avg: "     << stats_tf_   .latency.mean    * 1000.0 << "\n";
  out << "  tf_max: "     << stats_tf_   .latency.max_val * 1000.0 << "\n";
  out << "  tf_std: "     << stats_tf_   .latency.stddev() * 1000.0 << "\n\n";

  out << "warnings: " << warning_count_ << "\n";
  out << "track_notes: \"\"   # fill in by hand after the run\n";
  return out.good();
}

std::string HealthMonitorNode::make_timestamp_dir_name(const rclcpp::Time & t)
{
  // Build run-YYYY-MM-DD_HH-MM-SS from the launch wall-clock. Stable for the
  // lifetime of the process; never re-rolled.
  const std::time_t epoch = static_cast<std::time_t>(t.seconds());
  std::tm tm_local{};
  localtime_r(&epoch, &tm_local);
  std::ostringstream os;
  os << "run-"
     << std::put_time(&tm_local, "%Y-%m-%d_%H-%M-%S");
  return os.str();
}

}  // namespace bfmc_system
