#ifndef HEALTH_MONITOR_CPP__HEALTH_MONITOR_NODE_HPP_
#define HEALTH_MONITOR_CPP__HEALTH_MONITOR_NODE_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/lane_geometry.hpp"
#include "bfmc_msgs/msg/object_array.hpp"

namespace bfmc_system
{

// Welford online statistics (single-pass, numerically stable mean + variance).
// Used for scalar streams (yaw_bias, lane_heading) and for per-topic latency.
struct OnlineStats
{
  std::uint64_t n{0};
  double mean{0.0};
  double m2{0.0};
  double max_val{0.0};

  void add(double v)
  {
    ++n;
    const double d = v - mean;
    mean += d / static_cast<double>(n);
    m2 += d * (v - mean);
    if (n == 1 || std::abs(v) > max_val) max_val = std::abs(v);
  }

  double variance() const { return n > 1 ? m2 / static_cast<double>(n - 1) : 0.0; }
  double stddev()   const { return std::sqrt(variance()); }
  void reset() { n = 0; mean = 0.0; m2 = 0.0; max_val = 0.0; }
};

// Per-topic arrival bookkeeping. arrival_times holds wall-clock seconds in the
// rolling rate window; latency aggregates header.stamp→receive deltas when the
// upstream message carries a usable stamp.
struct TopicStats
{
  std::deque<double> arrival_times;        // wall-clock seconds, trimmed to window
  double last_arrival_sec{0.0};
  std::uint64_t total{0};
  std::uint64_t stale_count{0};
  OnlineStats latency;                     // seconds; only populated when stamp is valid
  double stale_threshold_s{0.2};

  double rate(double window_s) const
  {
    if (window_s <= 0.0 || arrival_times.empty()) return 0.0;
    // Rate = count / window_s; if the window is not yet full this under-reports
    // for the first window_s of life, which is fine for the warm-up period.
    return static_cast<double>(arrival_times.size()) / window_s;
  }

  void trim(double now_sec, double window_s)
  {
    const double cutoff = now_sec - window_s;
    while (!arrival_times.empty() && arrival_times.front() < cutoff) arrival_times.pop_front();
  }
};

class HealthMonitorNode : public rclcpp::Node
{
public:
  explicit HealthMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HealthMonitorNode() override;

private:
  // ---- callbacks ----
  void on_state         (const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void on_tf            (const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void on_correction    (const std_msgs::msg::Float64::SharedPtr msg);
  void on_lane_heading  (const std_msgs::msg::Float64::SharedPtr msg);
  void on_lane_geom     (const bfmc_msgs::msg::LaneGeometry::SharedPtr msg);
  void on_imu           (const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_image         (const sensor_msgs::msg::Image::SharedPtr msg);
  void on_tracked       (const bfmc_msgs::msg::ObjectArray::SharedPtr msg);

  // ---- timers ----
  void metrics_tick();
  void summary_tick();

  // ---- helpers ----
  void register_arrival(TopicStats & ts, double stamp_sec_or_zero);
  void detect_stale(TopicStats & ts);
  bool write_summary_yaml() const;
  static std::string make_timestamp_dir_name(const rclcpp::Time & t);

  // ---- subs ----
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr   sub_state_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr        sub_tf_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr          sub_correction_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr          sub_lane_heading_;
  rclcpp::Subscription<bfmc_msgs::msg::LaneGeometry>::SharedPtr    sub_lane_geom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         sub_image_;
  rclcpp::Subscription<bfmc_msgs::msg::ObjectArray>::SharedPtr     sub_tracked_;

  // ---- pubs ----
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   metrics_pub_;

  // ---- timers ----
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  // ---- per-topic stats ----
  TopicStats stats_state_;
  TopicStats stats_tf_;
  TopicStats stats_correction_;
  TopicStats stats_lane_heading_;
  TopicStats stats_lane_geom_;
  TopicStats stats_imu_;
  TopicStats stats_image_;
  TopicStats stats_tracked_;        // Phase 3.1: /perception/tracked_objects

  // ---- value-stream stats ----
  OnlineStats yaw_bias_stats_;
  OnlineStats lane_heading_stats_;

  // ---- counters ----
  std::uint64_t warning_count_{0};

  // ---- runtime parameters ----
  double rate_window_s_{5.0};
  double metrics_period_s_{1.0};
  double summary_period_s_{5.0};
  std::string validation_output_dir_;     // empty ⇒ no YAML written
  std::string run_dir_;                   // resolved once on construction

  // ---- launch wall-clock for "duration_s" in summary ----
  rclcpp::Time launch_time_;

  // ---- fixed metrics-array layout (matches docs/runtime_validation.md) ----
  // 0  vehicle_state_hz       8   lane_heading_mean       16  warning_count
  // 1  tf_hz                  9   lane_heading_std        17  uptime_s
  // 2  camera_hz             10   stale_state_count       18  tracked_objects_hz   (Phase 3.1)
  // 3  lane_hz               11   stale_tf_count          19  stale_tracked_count  (Phase 3.1)
  // 4  imu_hz                12   stale_camera_count
  // 5  correction_hz         13   stale_lane_count
  // 6  yaw_bias_mean         14   stale_imu_count
  // 7  yaw_bias_std          15   imu_latency_max_ms
  // APPEND-ONLY: indices 0..17 frozen by check_run.sh and PR contracts.
  static constexpr std::size_t METRICS_LAYOUT_SIZE = 20;
};

}  // namespace bfmc_system

#endif  // HEALTH_MONITOR_CPP__HEALTH_MONITOR_NODE_HPP_
