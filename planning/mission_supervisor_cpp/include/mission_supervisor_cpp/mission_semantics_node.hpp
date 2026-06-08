#ifndef MISSION_SUPERVISOR_CPP__MISSION_SEMANTICS_NODE_HPP_
#define MISSION_SUPERVISOR_CPP__MISSION_SEMANTICS_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"

#include "mission_supervisor_cpp/mission_graph.hpp"

namespace mission_supervisor_cpp
{

// Per-region dispatch row (loaded from competition_behavior_overrides.yaml +
// competition_speed_profiles.yaml).
struct RegionDispatch
{
  std::string maneuver{"LANE_KEEP"};
  double      speed{0.25};
  std::string nav_flag;            // forwarded onto /navigation/flags
};

// Per-region blend weights (loaded from competition_risk_zones.yaml).
struct BlendWeights
{
  double loc{0.4};
  double lane{0.4};
  double graph{0.2};
  double imu_yaw{0.0};
};

// Race-day central semantic provider.
//
// Inputs:
//   • 5 YAMLs from share/mission_supervisor_cpp/config/
//   • GraphML (defaulted to global_path_planner_cpp's share install)
//   • /vehicle/state             — for nearest-mission-node search
//   • /fusion/imu_confidence     — fed into blended confidence
//   • /perception/lane_geometry  — confidence channel only (no pose blend here)
//
// Outputs:
//   /planning/global_path              (nav_msgs/Path)  — authoritative mission path
//   /planning/competition_path         (nav_msgs/Path)  — duplicate, for visualization tooling
//   /planning/competition_markers      (MarkerArray)    — RViz colored line + current-node sphere
//   /navigation/flags                  (String)         — region nav_flag string
//   /planning/current_node             (Int32)
//   /planning/current_region           (String)
//   /planning/current_behavior         (String)
//   /planning/current_speed_limit      (Float64)
//   /planning/mission_progress         (Float64) [0..1]
//   /planning/mission_score_estimate   (Float64) [0..100]
//   /planning/localization_confidence  (Float64) [0..1]
class MissionSemanticsNode : public rclcpp::Node
{
public:
  explicit MissionSemanticsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Configuration parsing.
  bool load_mission_(const std::string & path);
  bool load_semantics_(const std::string & path);
  bool load_speed_profiles_(const std::string & path);
  bool load_risk_zones_(const std::string & path);
  bool load_behavior_overrides_(const std::string & path);

  // Mission validation against the loaded graph. Logs errors but never throws —
  // race-day prefers a degraded mission over a no-go.
  void validate_mission_();

  // Path / marker construction (done once at startup, republished by the timer).
  void build_competition_path_();
  void build_marker_array_();

  // Region lookup helpers.
  const std::string & region_of_node_(const std::string & node_id) const;
  const RegionDispatch & dispatch_of_region_(const std::string & region) const;
  const BlendWeights   & blend_of_region_(const std::string & region) const;

  // Timer tick — publishes everything at supervisor_hz_.
  void tick_();

  // Callbacks.
  void on_state_     (const bfmc_msgs::msg::VehicleState::SharedPtr);
  void on_imu_conf_  (const std_msgs::msg::Float64::SharedPtr);

  // Subscriptions.
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       sub_imu_conf_;

  // Publishers — race-day topics.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_global_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_competition_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_nav_flag_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            pub_current_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_current_region_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_current_behavior_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_current_speed_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_progress_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_score_est_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_loc_conf_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Loaded data.
  MissionGraph                                          graph_;
  std::vector<std::string>                              mission_path_;       // ordered node ids
  std::unordered_map<std::string, std::string>          node_region_;        // node-id → region
  std::unordered_map<std::string, RegionDispatch>       region_dispatch_;
  std::unordered_map<std::string, BlendWeights>         region_blend_;
  RegionDispatch                                        unknown_dispatch_;
  std::string                                           default_region_{"normal"};
  double                                                v_floor_{0.05};
  double                                                v_cap_{0.85};
  double                                                loc_low_thresh_{0.20};
  double                                                graceful_decay_s_{5.0};

  // Pre-built artifacts.
  nav_msgs::msg::Path                  competition_path_msg_;   // built once at startup
  visualization_msgs::msg::MarkerArray markers_msg_;            // built once at startup

  // Runtime state.
  std::mutex   state_mtx_;
  bool         have_state_{false};
  double       state_x_{0.0}, state_y_{0.0};
  double       state_conf_{0.0};
  double       imu_conf_{1.0};
  std::size_t  current_mission_idx_{0};
  std::string  current_region_{"normal"};
  std::string  last_nav_flag_published_;
  double       last_good_blended_conf_{1.0};
  rclcpp::Time last_state_t_{0, 0, RCL_ROS_TIME};

  // Hysteresis for the nearest-mission-node search. Without this, the index
  // flickers between two adjacent nodes (e.g. 399 ↔ 400) when the vehicle
  // crosses their bisector. We commit to a new index only if it's clearly
  // better OR if it has stayed best for >hys_persist_s_.
  std::size_t  hys_candidate_idx_{0};
  rclcpp::Time hys_candidate_t_{0, 0, RCL_ROS_TIME};
  double       hys_distance_margin_m_{0.30};
  double       hys_persist_s_       {0.50};

  // Validation results (kept so we can log + reflect them on a debug topic).
  std::vector<std::string> validation_errors_;
  bool                     mission_valid_{false};

  // Params.
  double      supervisor_hz_   {10.0};
  std::string graphml_path_;
  std::string config_dir_;
  std::string mission_yaml_;
  std::string semantics_yaml_;
  std::string speed_yaml_;
  std::string risk_yaml_;
  std::string overrides_yaml_;
};

}  // namespace mission_supervisor_cpp

#endif  // MISSION_SUPERVISOR_CPP__MISSION_SEMANTICS_NODE_HPP_
