#ifndef RUNTIME_TELEMETRY_CPP__RUNTIME_TELEMETRY_NODE_HPP_
#define RUNTIME_TELEMETRY_CPP__RUNTIME_TELEMETRY_NODE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace runtime_telemetry_cpp
{

// Phase Final P6 — Jetson-side runtime telemetry.
//
// Reads cheap sysfs files only — NO tegrastats subprocess, NO i2c probing.
// Designed to add ~0.05 % CPU overhead at 1 Hz.
//
// Sources:
//   /proc/stat                              → per-CPU + aggregate %busy
//   /proc/meminfo                           → MemTotal / MemAvailable
//   /sys/class/thermal/thermal_zone*/{type,temp}
//                                           → CPU/GPU/SoC/Tj temps (millidegC)
//
// Output topic (Reliable, 1 Hz):
//   /system/runtime_health  (std_msgs/String, JSON):
//     {
//       "cpu_busy_pct":  17.4,
//       "ram_used_pct":  43.2,
//       "temp_cpu_c":    44.9,
//       "temp_gpu_c":    44.8,
//       "temp_tj_c":     45.1,
//       "temp_max_c":    45.1,
//       "brownout_flag": false,
//       "tegra_throttling": "unknown"
//     }
//
// Brownout heuristic: temp_max_c ≥ 85 °C OR cpu_busy_pct ≥ 95% for ≥ N ticks.
// This is a SOFT signal — race-day operator decides whether to throttle.
class RuntimeTelemetryNode : public rclcpp::Node
{
public:
  explicit RuntimeTelemetryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CpuSample {
    uint64_t user{0}, nice{0}, system{0}, idle{0}, iowait{0}, irq{0}, softirq{0}, steal{0};
    uint64_t total() const { return user + nice + system + idle + iowait + irq + softirq + steal; }
    uint64_t busy()  const { return user + nice + system + irq + softirq + steal; }
  };

  void tick();
  bool read_cpu_(CpuSample & out) const;
  bool read_mem_(double & used_pct_out) const;
  void read_thermal_(double & cpu_c, double & gpu_c, double & tj_c, double & max_c) const;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                        timer_;

  // CPU rolling sample.
  CpuSample prev_cpu_{};
  bool      have_prev_cpu_{false};

  // Brownout debouncing.
  int       brownout_strikes_{0};

  // Params.
  double tick_period_s_      {1.0};
  double brownout_temp_c_    {85.0};
  double brownout_cpu_pct_   {95.0};
  int    brownout_strikes_max_{3};
};

}  // namespace runtime_telemetry_cpp

#endif  // RUNTIME_TELEMETRY_CPP__RUNTIME_TELEMETRY_NODE_HPP_
