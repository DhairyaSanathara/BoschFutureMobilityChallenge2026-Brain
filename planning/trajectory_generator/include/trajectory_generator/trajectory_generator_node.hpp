// Trajectory Generator – BFMC 2026 Competition Grade
// Team OdyssiX
//
// Consumes local_path (JSON/String), applies forward+backward velocity
// smoothing with physical acceleration constraints, time-parameterises
// the result, converts curvature -> steering angle, and publishes
// the final strict ROS trajectory message.

#ifndef TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_NODE_HPP_
#define TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_NODE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <bfmc_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

namespace trajectory_generator
{

// ================================================================
//  PRE-ALLOCATED POINT STRUCTS (zero dynamic alloc in hot path)
// ================================================================

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double target_speed{0.0};
};

struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double time_from_start{0.0};
  double steering_angle{0.0};
};

// ================================================================
//  NODE
// ================================================================

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  explicit TrajectoryGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- parameters ----
  double wheelbase_;
  double a_max_;
  double planner_rate_hz_;
  double epsilon_velocity_;

  // ---- pre-allocated buffers (avoid malloc in hot loop) ----
  static constexpr std::size_t kMaxPoints = 128;
  std::vector<PathPoint>       path_buf_;
  std::vector<double>          arc_buf_;      // arc lengths
  std::vector<double>          vel_fwd_buf_;  // forward pass velocities
  std::vector<double>          vel_bwd_buf_;  // backward pass velocities
  std::vector<TrajectoryPoint> traj_buf_;

  // ---- architecture state forwarding ----
  std::string current_behavior_{"STARTUP.INIT"};
  std::string current_frame_{"vehicle"};

  // ---- ROS interfaces ----
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_path_;
  rclcpp::Publisher<bfmc_msgs::msg::Trajectory>::SharedPtr pub_traj_;
  rclcpp::TimerBase::SharedPtr                           timer_;

  // ---- cached input ----
  std_msgs::msg::String::SharedPtr last_path_msg_;
  bool new_path_received_{false};

  // ---- callbacks ----
  void on_path(const std_msgs::msg::String::SharedPtr msg);
  void on_timer();

  // ---- pipeline stages ----
  bool parse_path(const std::string & json_str, std::size_t & out_n);
  void compute_arc_lengths(std::size_t n);
  void forward_pass(std::size_t n);
  void backward_pass(std::size_t n);
  void merge_velocity(std::size_t n);
  void time_parameterise(std::size_t n);
  void publish_trajectory(std::size_t n);
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR__TRAJECTORY_GENERATOR_NODE_HPP_
