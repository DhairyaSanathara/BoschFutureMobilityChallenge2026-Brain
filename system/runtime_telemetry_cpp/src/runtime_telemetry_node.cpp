#include "runtime_telemetry_cpp/runtime_telemetry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace runtime_telemetry_cpp
{

RuntimeTelemetryNode::RuntimeTelemetryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("runtime_telemetry_node", options)
{
  tick_period_s_        = declare_parameter<double>("tick_period_s",        tick_period_s_);
  brownout_temp_c_      = declare_parameter<double>("brownout_temp_c",      brownout_temp_c_);
  brownout_cpu_pct_     = declare_parameter<double>("brownout_cpu_pct",     brownout_cpu_pct_);
  brownout_strikes_max_ = declare_parameter<int>   ("brownout_strikes_max", brownout_strikes_max_);

  auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  pub_ = create_publisher<std_msgs::msg::String>("/system/runtime_health", rel);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(tick_period_s_ * 1000.0)),
    std::bind(&RuntimeTelemetryNode::tick, this));

  RCLCPP_INFO(get_logger(),
    "runtime_telemetry_node: %.1f Hz, brownout >= %.0f C OR >= %.0f%% CPU x %d ticks",
    1.0/tick_period_s_, brownout_temp_c_, brownout_cpu_pct_, brownout_strikes_max_);
}

bool RuntimeTelemetryNode::read_cpu_(CpuSample & out) const
{
  std::ifstream f("/proc/stat");
  if (!f.is_open()) return false;
  std::string label;
  f >> label;
  if (label != "cpu") return false;
  f >> out.user >> out.nice >> out.system >> out.idle
    >> out.iowait >> out.irq >> out.softirq >> out.steal;
  return f.good() || f.eof();
}

bool RuntimeTelemetryNode::read_mem_(double & used_pct_out) const
{
  std::ifstream f("/proc/meminfo");
  if (!f.is_open()) return false;
  uint64_t total_kb = 0, avail_kb = 0;
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("MemTotal:", 0) == 0) {
      std::stringstream ss(line.substr(9));
      ss >> total_kb;
    } else if (line.rfind("MemAvailable:", 0) == 0) {
      std::stringstream ss(line.substr(13));
      ss >> avail_kb;
    }
    if (total_kb > 0 && avail_kb > 0) break;
  }
  if (total_kb == 0) return false;
  used_pct_out = 100.0 * (1.0 - static_cast<double>(avail_kb) / static_cast<double>(total_kb));
  return true;
}

void RuntimeTelemetryNode::read_thermal_(double & cpu_c, double & gpu_c,
                                          double & tj_c, double & max_c) const
{
  cpu_c = gpu_c = tj_c = -1.0;
  max_c = -1.0;

  DIR * dir = ::opendir("/sys/class/thermal");
  if (dir == nullptr) return;
  dirent * ent = nullptr;
  while ((ent = ::readdir(dir)) != nullptr) {
    std::string name(ent->d_name);
    if (name.rfind("thermal_zone", 0) != 0) continue;

    const std::string base = std::string("/sys/class/thermal/") + name;
    std::ifstream ft(base + "/type");
    std::ifstream fv(base + "/temp");
    if (!ft.is_open() || !fv.is_open()) continue;
    std::string type;
    int millideg = 0;
    ft >> type;
    fv >> millideg;
    if (!fv.good() || millideg <= 0) continue;
    const double c = millideg / 1000.0;
    if (c > max_c) max_c = c;
    if      (type == "cpu-thermal" || type == "CPU-therm") cpu_c = c;
    else if (type == "gpu-thermal" || type == "GPU-therm") gpu_c = c;
    else if (type == "tj-thermal")                         tj_c  = c;
  }
  ::closedir(dir);
}

void RuntimeTelemetryNode::tick()
{
  CpuSample cur{};
  const bool cpu_ok = read_cpu_(cur);

  double cpu_busy_pct = -1.0;
  if (cpu_ok && have_prev_cpu_) {
    const uint64_t d_total = cur.total() - prev_cpu_.total();
    const uint64_t d_busy  = cur.busy()  - prev_cpu_.busy();
    if (d_total > 0) {
      cpu_busy_pct = 100.0 * static_cast<double>(d_busy) / static_cast<double>(d_total);
    }
  }
  if (cpu_ok) { prev_cpu_ = cur; have_prev_cpu_ = true; }

  double ram_used_pct = -1.0;
  read_mem_(ram_used_pct);

  double temp_cpu = -1.0, temp_gpu = -1.0, temp_tj = -1.0, temp_max = -1.0;
  read_thermal_(temp_cpu, temp_gpu, temp_tj, temp_max);

  const bool over_temp = (temp_max >= brownout_temp_c_);
  const bool over_cpu  = (cpu_busy_pct >= brownout_cpu_pct_);
  if (over_temp || over_cpu) {
    brownout_strikes_ = std::min(brownout_strikes_ + 1, 1000);
  } else {
    brownout_strikes_ = std::max(brownout_strikes_ - 1, 0);
  }
  const bool brownout = (brownout_strikes_ >= brownout_strikes_max_);

  // Build compact JSON manually — pulling in a JSON library for one publisher
  // would dwarf the rest of this node.
  std::ostringstream js;
  js << std::fixed << std::setprecision(1);
  js << "{"
     << "\"cpu_busy_pct\":"   << cpu_busy_pct << ","
     << "\"ram_used_pct\":"   << ram_used_pct << ","
     << "\"temp_cpu_c\":"     << temp_cpu     << ","
     << "\"temp_gpu_c\":"     << temp_gpu     << ","
     << "\"temp_tj_c\":"      << temp_tj      << ","
     << "\"temp_max_c\":"     << temp_max     << ","
     << "\"brownout_strikes\":" << brownout_strikes_ << ","
     << "\"brownout_flag\":"  << (brownout ? "true" : "false")
     << "}";

  std_msgs::msg::String msg;
  msg.data = js.str();
  pub_->publish(msg);

  if (brownout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "BROWNOUT SOFT-FLAG: temp_max=%.1fC cpu=%.1f%% strikes=%d",
      temp_max, cpu_busy_pct, brownout_strikes_);
  }
}

}  // namespace runtime_telemetry_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<runtime_telemetry_cpp::RuntimeTelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
