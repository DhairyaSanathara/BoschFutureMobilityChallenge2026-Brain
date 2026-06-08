#ifndef GLOBAL_PATH_PLANNER_CPP__PLANNER_NODE_HPP_
#define GLOBAL_PATH_PLANNER_CPP__PLANNER_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/path.hpp"

#include "bfmc_msgs/msg/vehicle_state.hpp"

#include "global_path_planner_cpp/graph.hpp"
#include "global_path_planner_cpp/planner.hpp"
#include "global_path_planner_cpp/zone_map.hpp"

namespace global_path_planner_cpp
{

class GlobalPathPlannerNode : public rclcpp::Node
{
public:
  explicit GlobalPathPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void on_goal_command(const std_msgs::msg::String::SharedPtr msg);
  void republish_timer_tick();

  // Compute (or re-compute) the global path from the current nearest node to
  // `goal_node_`. Updates `current_path_` + publishes nav_msgs/Path. Returns
  // false if planning failed (logged inside).
  bool plan_to_goal_();

  // Publish the cached current_path_. Caller is responsible for ensuring
  // current_path_ is fresh.
  void publish_current_path_();

  // -------- ROS interfaces --------
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_path_;
  // Phase Final: per-tick navigation hint for the behavior planner. Sourced
  // from zones.yaml + current nearest node + path lookahead. Values:
  //   "intersection" | "roundabout" | "parking" | "trafficlight" |
  //   "highway"      | "highway_entry" | "highway_exit" | "" (none)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_nav_flag_;
  // Debug topics (always on — tiny payload, race-day useful).
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_dbg_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_dbg_goal_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_dbg_dev_;
  rclcpp::TimerBase::SharedPtr                                  republish_timer_;

  // -------- Algorithmic state --------
  Graph                       graph_;
  ZoneMap                     zones_;            // Phase Final
  std::string                 start_node_;       // fallback manual start (runtime patch)
  std::string                 goal_node_;
  Algo                        algo_{Algo::AStar};
  std::vector<std::string>    current_path_;
  std::string                 current_nearest_id_;
  std::string                 last_nav_flag_;    // Phase Final: dedup publishes

  // -------- Configuration --------
  std::string graphml_path_;
  std::string zones_yaml_path_;          // Phase Final
  std::string frame_id_{"map"};
  double      replan_deviation_m_{0.4};
  double      republish_period_s_{1.0};
  bool        graph_ready_{false};
  // Phase Final P-mission: when true, the planner still LOADS the graph
  // (state callbacks + debug pubs still work) but skips its own publishes
  // on /planning/global_path and /navigation/flags. Used when the
  // mission_supervisor takes authoritative ownership of those topics.
  bool        disable_pub_{false};

  // -------- Replan storm guard (race-day P1) ------------------------------
  // A deviation-driven replan only fires when ALL of:
  //   1) deviation > replan_deviation_m_ for ≥ replan_consecutive_ samples
  //   2) vehicle speed > replan_min_speed_mps_
  //   3) at least replan_min_interval_s_ since the previous replan
  // Catastrophic replans (no path yet, or path start ≠ nearest node) bypass
  // the guard — they are one-shot recovery events, not storm sources.
  int    replan_consecutive_{5};
  double replan_min_speed_mps_{0.1};
  double replan_min_interval_s_{3.0};
  int    dev_over_count_{0};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  float       last_speed_mps_{0.0f};

  // -------- Last-seen vehicle pose (cached for goal-command + republish) ----
  std::atomic<bool> have_state_{false};
  double last_x_{0.0};
  double last_y_{0.0};
};

}  // namespace global_path_planner_cpp

#endif  // GLOBAL_PATH_PLANNER_CPP__PLANNER_NODE_HPP_
