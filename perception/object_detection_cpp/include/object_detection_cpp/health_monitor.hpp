#ifndef OBJECT_DETECTION_CPP__HEALTH_MONITOR_HPP_
#define OBJECT_DETECTION_CPP__HEALTH_MONITOR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <rclcpp/logger.hpp>

namespace object_detection_cpp
{

class HealthMonitor
{
public:
  HealthMonitor(rclcpp::Logger logger,
                int publish_rate_hz,
                double max_log_rate_hz,
                size_t latency_window = 300);

  void start();
  void record_inference(double latency_ms);
  void record_error(const std::string & msg);
  void log_periodic_summary(size_t interval_frames = 450);

private:
  rclcpp::Logger logger_;
  size_t latency_window_;
  double expected_period_ms_;
  double max_log_interval_s_;

  std::deque<double> latencies_;
  uint64_t total_frames_{0};
  uint64_t dropped_frames_{0};
  uint64_t error_count_{0};
  uint64_t suppressed_errors_{0};

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_frame_time_{};
  std::chrono::steady_clock::time_point last_error_log_time_{};
  bool has_last_frame_{false};
};

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__HEALTH_MONITOR_HPP_
