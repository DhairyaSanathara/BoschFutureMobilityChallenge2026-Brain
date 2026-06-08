#ifndef LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/lane_geometry.hpp"
#include "bfmc_msgs/msg/object_array.hpp"

#include "local_path_planner_cpp/local_planner.hpp"

namespace local_path_planner_cpp
{

class LocalPathPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPathPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /* ---- Callbacks ---- */
  void on_state    (const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void on_lane     (const bfmc_msgs::msg::LaneGeometry::SharedPtr msg);
  void on_behavior (const std_msgs::msg::String::SharedPtr msg);
  void on_target_v (const std_msgs::msg::Float32::SharedPtr msg);
  void on_objects  (const bfmc_msgs::msg::ObjectArray::SharedPtr msg);

  /* ---- Tick ---- */
  void tick();

  // Build the lookahead station grid (0, L/N-1, 2·L/N-1, …, L).
  std::vector<double> make_stations(double L, int N) const;

  // Generate per-behavior-state path. Returns (points, base_target_speed).
  std::vector<TrajectoryPoint> generate_(const std::string & behavior,
                                         const Poly3 & lane,
                                         const std::vector<double> & stations,
                                         double speed,
                                         double target_speed);

  // Curvature speed limit (a_lat = v² · |κ|). Mutates points[].velocity.
  void apply_curvature_speed_limit_(std::vector<TrajectoryPoint> & pts) const;

  // Phase B: scan /perception/tracked_objects. If any object's (x, y) lies
  // within `obstacle_radius_` of any path point AND within forward look,
  // apply decel-to-stop down to v=0 at that station onwards. Mutates pts.
  // Returns true if any obstacle was clipped (used to lower path_confidence).
  bool apply_obstacle_clip_(std::vector<TrajectoryPoint> & pts);

  // Compose the JSON string (matching the Python schema).
  std::string to_json_(const std::string & behavior,
                       const std::vector<TrajectoryPoint> & pts) const;

  /* ---- ROS interfaces ---- */
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr  sub_state_;
  rclcpp::Subscription<bfmc_msgs::msg::LaneGeometry>::SharedPtr  sub_lane_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         sub_behavior_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        sub_target_v_;
  rclcpp::Subscription<bfmc_msgs::msg::ObjectArray>::SharedPtr   sub_objects_;
  // Race-day overtake: behavior_planner publishes a lateral-offset
  // command (m, +ve = left) each tick. Local planner adds it to every
  // TrajectoryPoint.y to execute the lane change.
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        sub_lat_offset_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_path_conf_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  /* ---- Cached inputs ---- */
  mutable std::mutex cache_mtx_;
  bool   have_state_{false};
  bfmc_msgs::msg::VehicleState  latest_state_;
  bfmc_msgs::msg::LaneGeometry  latest_lane_;
  bfmc_msgs::msg::ObjectArray   latest_objects_;
  bool                          have_lane_   {false};
  bool                          have_objects_{false};
  std::string                   behavior_state_{"LANE_KEEP"};
  double                        target_speed_  {0.0};
  // Atomic single-writer (subscriber) / single-reader (tick) pattern —
  // no mutex needed for a scalar.
  std::atomic<double>           lateral_offset_cmd_{0.0};

  /* ---- Parameters ---- */
  double planner_hz_{20.0};
  double lookahead_m_{2.0};
  int    num_points_{40};
  double a_max_{1.8};
  double a_lat_max_{0.8};            // m/s² lateral cap → v ≤ sqrt(a_lat/|κ|)
  double min_curve_speed_{0.05};
  double overtake_lateral_m_{0.30};
  double overtake_distance_m_{1.50};
  // Phase B: collision check window.
  double obstacle_radius_{0.30};     // m — clip path if any obj within this radius
  double obstacle_lookahead_max_{2.5}; // m — ignore obstacles farther than this
  // Fallback poly used when lane confidence is too low or lane is stale.
  // y = a + b*x + c*x², default = straight ahead (all zero).
  double fallback_poly_a_{0.0};
  double fallback_poly_b_{0.0};
  double fallback_poly_c_{0.0};
  double lane_min_confidence_{0.20}; // below this → fallback poly
  double stale_lane_s_   {0.5};
  double stale_state_s_  {0.2};
  double stale_objects_s_{0.5};

  /* ---- Cached time markers for stale gates ---- */
  rclcpp::Time last_state_time_  {0,0,RCL_ROS_TIME};
  rclcpp::Time last_lane_time_   {0,0,RCL_ROS_TIME};
  rclcpp::Time last_objects_time_{0,0,RCL_ROS_TIME};
};

}  // namespace local_path_planner_cpp

#endif  // LOCAL_PATH_PLANNER_CPP__LOCAL_PLANNER_NODE_HPP_
