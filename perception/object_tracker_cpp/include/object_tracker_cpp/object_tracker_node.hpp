#ifndef OBJECT_TRACKER_CPP__OBJECT_TRACKER_NODE_HPP_
#define OBJECT_TRACKER_CPP__OBJECT_TRACKER_NODE_HPP_

#include <cstdint>
#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "bfmc_msgs/msg/object_array.hpp"

#include "object_tracker_cpp/byte_tracker.hpp"

namespace object_tracker_cpp
{

// ID-switch heuristic: when a confirmed/lost track dies, log its last known
// position + class for a few frames; if a new birth of the same class spawns
// within ID_SWITCH_RADIUS metres while still in the window, count it as a
// likely ID switch. Cheap, no ground truth required.
struct DeathRecord
{
  std::uint16_t track_id;
  std::uint8_t  class_id;
  double        x;
  double        y;
  std::uint64_t frame_index;     // monotonic frame counter at time of death
};

class ObjectTrackerNode : public rclcpp::Node
{
public:
  explicit ObjectTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_objects(const bfmc_msgs::msg::ObjectArray::SharedPtr msg);

  // Tally ID switches by comparing this step's births against the rolling
  // death buffer. Side-effect: prunes expired death records.
  void update_id_switch_count_(std::uint64_t now_frame);

  rclcpp::Subscription<bfmc_msgs::msg::ObjectArray>::SharedPtr sub_;
  rclcpp::Publisher<bfmc_msgs::msg::ObjectArray>::SharedPtr    pub_;

  // Phase 3.1 debug publishers (always created; cost is < 1 KB/s aggregate).
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr           debug_track_count_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr debug_active_tracks_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          debug_latency_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr           debug_id_switch_pub_;

  std::unique_ptr<ByteTracker> tracker_;

  // Real-timestamp dt bookkeeping.
  rclcpp::Time   last_msg_stamp_{0, 0, RCL_ROS_TIME};
  bool           have_prev_stamp_{false};

  // Throughput / latency logging.
  rclcpp::Time   last_log_time_;
  std::uint64_t  msgs_in_window_{0};

  // ID-switch heuristic state.
  std::uint64_t                frame_index_{0};
  std::deque<DeathRecord>      recent_deaths_;
  std::uint32_t                id_switch_count_{0};

  // Output buffer reused across publishes to avoid per-tick allocation.
  bfmc_msgs::msg::ObjectArray out_msg_;

  // Topic config — runtime parameters so a launch override can repoint
  // without recompiling the node.
  std::string input_topic_{"/perception/objects"};
  std::string output_topic_{"/perception/tracked_objects"};

  // dt clamp window for the constant-velocity KF prediction. Spec'd by
  // Phase 3.1 (0.01 ≤ dt ≤ 0.1). dt is derived from the upstream
  // ObjectArray.stamp delta, not from any wall clock or fixed frame interval.
  static constexpr double DT_MIN = 0.01;
  static constexpr double DT_MAX = 0.10;

  // ID-switch heuristic constants.
  static constexpr double        ID_SWITCH_RADIUS    = 1.0;   // m
  static constexpr std::uint64_t ID_SWITCH_WINDOW    = 5;     // frames
};

}  // namespace object_tracker_cpp

#endif  // OBJECT_TRACKER_CPP__OBJECT_TRACKER_NODE_HPP_
