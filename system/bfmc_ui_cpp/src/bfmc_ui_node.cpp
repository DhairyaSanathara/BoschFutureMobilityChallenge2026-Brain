// Race-day BFMC UI aggregator.
//
// Subscribes to all topics the external dashboard needs in one place and
// republishes a single compact JSON on /ui/status at 5 Hz. Passive — does
// not feed back into the stack. The JSON contains exactly the fields the
// race-day brief asks for:
//
//   speed         (m/s, EKF speed)
//   position      ({x, y} in map frame)
//   yaw           (rad)
//   current_node  (graph node id, latched)
//   current_region (mission semantics region, latched)
//   vehicle_ready (bool)
//   system_state  (named state from system_state_machine)
//   throttle      (last clamped /control/command)
//   steering      (rad)
//   brake
//   behavior      (FSM state from /planning/behavior_state)
//   mission_path  ({count, first, last})
//   target_speed  (m/s)
//
// Format choice: std_msgs/String JSON. Matches the legacy BFMC dashboard
// contract and is trivial for any consumer (Python / web / Qt) to parse.

#include <atomic>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"

#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/control_command.hpp"

namespace bfmc_ui_cpp
{

class BfmcUiNode : public rclcpp::Node
{
public:
  explicit BfmcUiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("bfmc_ui_node", options)
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 5.0);

    auto rel = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    auto be  = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    sub_state_ = create_subscription<bfmc_msgs::msg::VehicleState>(
      "/vehicle/state", rel,
      [this](bfmc_msgs::msg::VehicleState::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (std::isfinite(m->pose.position.x) && std::isfinite(m->pose.position.y)) {
          x_ = m->pose.position.x; y_ = m->pose.position.y;
        }
        yaw_   = m->yaw;
        speed_ = m->speed;
        confidence_ = m->confidence;
        have_state_ = true;
      });

    sub_ready_ = create_subscription<std_msgs::msg::Bool>(
      "/system/vehicle_ready", rel,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        vehicle_ready_ = m->data;
      });

    sub_sys_state_ = create_subscription<std_msgs::msg::String>(
      "/system/system_state", rel,
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        system_state_ = m->data;
      });

    sub_command_ = create_subscription<bfmc_msgs::msg::ControlCommand>(
      "/control/command", rel,
      [this](bfmc_msgs::msg::ControlCommand::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        throttle_ = m->throttle;
        brake_    = m->brake;
        steering_ = m->steering_angle;
      });

    sub_behavior_ = create_subscription<std_msgs::msg::String>(
      "/planning/behavior_state", rel,
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        behavior_ = m->data;
      });

    sub_target_speed_ = create_subscription<std_msgs::msg::Float32>(
      "/planning/target_speed", rel,
      [this](std_msgs::msg::Float32::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        target_speed_ = m->data;
      });

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/global_path", latched,
      [this](nav_msgs::msg::Path::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        path_count_ = m->poses.size();
        if (!m->poses.empty()) {
          first_x_ = m->poses.front().pose.position.x;
          first_y_ = m->poses.front().pose.position.y;
          last_x_  = m->poses.back().pose.position.x;
          last_y_  = m->poses.back().pose.position.y;
        }
      });

    // /planning/debug/current_node — best-effort (debug topic).
    sub_node_ = create_subscription<std_msgs::msg::String>(
      "/planning/debug/current_node", be,
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        current_node_ = m->data;
      });

    // /navigation/flags — region / nav intent (mission_supervisor or planner).
    sub_region_ = create_subscription<std_msgs::msg::String>(
      "/navigation/flags", rel,
      [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        current_region_ = m->data;
      });

    pub_ = create_publisher<std_msgs::msg::String>("/ui/status", rel);

    const auto period_ms = static_cast<int>(1000.0 / std::max(0.1, publish_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&BfmcUiNode::tick_, this));

    RCLCPP_INFO(get_logger(),
      "bfmc_ui_node: publishing /ui/status @ %.1f Hz", publish_hz_);
  }

private:
  void tick_()
  {
    std::lock_guard<std::mutex> lk(mtx_);

    // Compact JSON, hand-built (no external dep). All numeric fields use
    // 3-decimal precision — good enough for a 1280×720 dashboard.
    std::ostringstream os;
    os.precision(3); os << std::fixed;
    os << "{"
       << "\"speed\":"        << speed_           << ","
       << "\"target_speed\":" << target_speed_    << ","
       << "\"throttle\":"     << throttle_        << ","
       << "\"brake\":"        << brake_           << ","
       << "\"steering\":"     << steering_        << ","
       << "\"yaw\":"          << yaw_             << ","
       << "\"confidence\":"   << confidence_      << ","
       << "\"position\":{"
       <<   "\"x\":" << x_ << ",\"y\":" << y_
       << "},"
       << "\"vehicle_ready\":" << (vehicle_ready_ ? "true" : "false") << ","
       << "\"system_state\":\"" << esc_(system_state_)   << "\","
       << "\"current_node\":\"" << esc_(current_node_)   << "\","
       << "\"current_region\":\"" << esc_(current_region_) << "\","
       << "\"behavior\":\""      << esc_(behavior_)       << "\","
       << "\"mission_path\":{"
       <<   "\"count\":" << path_count_ << ","
       <<   "\"first\":{\"x\":" << first_x_ << ",\"y\":" << first_y_ << "},"
       <<   "\"last\":{\"x\":"  << last_x_  << ",\"y\":" << last_y_  << "}"
       << "},"
       << "\"have_state\":" << (have_state_ ? "true" : "false")
       << "}";

    std_msgs::msg::String m;
    m.data = os.str();
    pub_->publish(m);
  }

  static std::string esc_(const std::string & s)
  {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
      switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        default:   out += c;
      }
    }
    return out;
  }

  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr   sub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            sub_ready_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_sys_state_;
  rclcpp::Subscription<bfmc_msgs::msg::ControlCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_behavior_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr         sub_target_speed_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            sub_path_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_region_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  double x_{0.0}, y_{0.0};
  double yaw_{0.0};
  double speed_{0.0};
  double confidence_{0.0};
  double throttle_{0.0}, brake_{0.0}, steering_{0.0};
  double target_speed_{0.0};
  bool   vehicle_ready_{false};
  bool   have_state_{false};
  std::string system_state_, current_node_, current_region_, behavior_;
  std::size_t path_count_{0};
  double first_x_{0.0}, first_y_{0.0}, last_x_{0.0}, last_y_{0.0};
  double publish_hz_{5.0};
};

}  // namespace bfmc_ui_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bfmc_ui_cpp::BfmcUiNode>());
  rclcpp::shutdown();
  return 0;
}
