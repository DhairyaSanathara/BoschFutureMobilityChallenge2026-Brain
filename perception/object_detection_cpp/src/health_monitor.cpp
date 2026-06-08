#include "object_detection_cpp/health_monitor.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

#include <rclcpp/logging.hpp>

namespace object_detection_cpp
{

HealthMonitor::HealthMonitor(
  rclcpp::Logger logger,
  int publish_rate_hz,
  double max_log_rate_hz,
  size_t latency_window)
: logger_(logger),
  latency_window_(latency_window),
  expected_period_ms_(1000.0 / static_cast<double>(std::max(1, publish_rate_hz))),
  max_log_interval_s_(max_log_rate_hz > 0.0 ? 1.0 / max_log_rate_hz : 1.0)
{
}

void HealthMonitor::start()
{
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "Health monitor started.");
}

void HealthMonitor::record_inference(double latency_ms)
{
  latencies_.push_back(latency_ms);
  if (latencies_.size() > latency_window_) {
    latencies_.pop_front();
  }
  total_frames_++;

  const auto now = std::chrono::steady_clock::now();
  if (has_last_frame_) {
    const double gap_ms =
      std::chrono::duration<double, std::milli>(now - last_frame_time_).count();
    if (gap_ms > expected_period_ms_ * 2.5) {
      dropped_frames_++;
    }
  }
  last_frame_time_ = now;
  has_last_frame_ = true;
}

void HealthMonitor::record_error(const std::string & msg)
{
  error_count_++;
  const auto now = std::chrono::steady_clock::now();
  const double since_last =
    std::chrono::duration<double>(now - last_error_log_time_).count();

  if (since_last >= max_log_interval_s_) {
    if (suppressed_errors_ > 0) {
      RCLCPP_ERROR(logger_, "%s (+%lu suppressed)",
                   msg.c_str(),
                   static_cast<unsigned long>(suppressed_errors_));
    } else {
      RCLCPP_ERROR(logger_, "%s", msg.c_str());
    }
    last_error_log_time_ = now;
    suppressed_errors_ = 0;
  } else {
    suppressed_errors_++;
  }
}

void HealthMonitor::log_periodic_summary(size_t interval_frames)
{
  if (total_frames_ == 0 || total_frames_ % interval_frames != 0) {
    return;
  }

  if (latencies_.empty()) {
    return;
  }

  std::vector<double> sorted(latencies_.begin(), latencies_.end());
  std::sort(sorted.begin(), sorted.end());

  const size_t n = sorted.size();
  const double avg =
    std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(n);
  const double p50 = sorted[static_cast<size_t>(n * 0.50)];
  const double p95 = sorted[static_cast<size_t>(n * 0.95)];

  const auto now = std::chrono::steady_clock::now();
  const double uptime =
    std::chrono::duration<double>(now - start_time_).count();
  const double fps =
    uptime > 0.0 ? static_cast<double>(total_frames_) / uptime : 0.0;

  RCLCPP_INFO(logger_,
              "HEALTH [%lu frames | %.0fs] fps=%.1f avg=%.1fms p50=%.1fms "
              "p95=%.1fms drops=%lu errs=%lu",
              static_cast<unsigned long>(total_frames_), uptime,
              fps, avg, p50, p95,
              static_cast<unsigned long>(dropped_frames_),
              static_cast<unsigned long>(error_count_));
}

}  // namespace object_detection_cpp
