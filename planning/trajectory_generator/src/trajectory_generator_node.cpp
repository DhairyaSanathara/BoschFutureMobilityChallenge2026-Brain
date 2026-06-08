// Trajectory Generator – BFMC 2026 Competition Grade
// Team OdyssiX
// Updated: Synced a_max to 1.2m/s^2 for NMPC Reference Consistency

#include "trajectory_generator/trajectory_generator_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace trajectory_generator
{

using json = nlohmann::json;

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const rclcpp::NodeOptions & options)
: Node("trajectory_generator_node", options)
{
  this->declare_parameter<double>("wheelbase", 0.26);
  this->declare_parameter<double>("a_max", 1.2); // SYNCED
  this->declare_parameter<double>("planner_rate_hz", 20.0);
  this->declare_parameter<double>("epsilon_velocity", 0.01);

  wheelbase_        = this->get_parameter("wheelbase").as_double();
  a_max_            = this->get_parameter("a_max").as_double();
  planner_rate_hz_  = this->get_parameter("planner_rate_hz").as_double();
  epsilon_velocity_ = this->get_parameter("epsilon_velocity").as_double();

  path_buf_.resize(kMaxPoints);
  arc_buf_.resize(kMaxPoints);
  vel_fwd_buf_.resize(kMaxPoints);
  vel_bwd_buf_.resize(kMaxPoints);
  traj_buf_.resize(kMaxPoints);

  auto rel_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);
  auto be_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort).durability(rclcpp::DurabilityPolicy::Volatile);

  sub_path_ = this->create_subscription<std_msgs::msg::String>(
    "/planning/local_path", rel_qos, std::bind(&TrajectoryGeneratorNode::on_path, this, std::placeholders::_1));

  pub_traj_ = this->create_publisher<bfmc_msgs::msg::Trajectory>("/planning/trajectory", be_qos);

  const double period_ms = 1000.0 / planner_rate_hz_;
  timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(period_ms * 1000.0)), std::bind(&TrajectoryGeneratorNode::on_timer, this));

  RCLCPP_INFO(this->get_logger(),
    "[TrajGen] Running at %.0f Hz | wheelbase=%.3f | a_max=%.2f | eps=%.4f",
    planner_rate_hz_, wheelbase_, a_max_, epsilon_velocity_);
}

void TrajectoryGeneratorNode::on_path(const std_msgs::msg::String::SharedPtr msg)
{
  last_path_msg_     = msg;
  new_path_received_ = true;
}

void TrajectoryGeneratorNode::on_timer()
{
  if (!last_path_msg_ || !new_path_received_) return;

  std::size_t n = 0;
  if (!parse_path(last_path_msg_->data, n)) return;

  new_path_received_ = false;

  if (n < 2) return;

  compute_arc_lengths(n);
  forward_pass(n);
  backward_pass(n);
  merge_velocity(n);
  time_parameterise(n);
  publish_trajectory(n);
}

bool TrajectoryGeneratorNode::parse_path(const std::string & json_str, std::size_t & out_n)
{
  try {
    auto j = json::parse(json_str);
    if (!j.contains("points") || !j["points"].is_array()) return false;

    current_behavior_ = j.value("behavior", "UNKNOWN");
    current_frame_    = j.value("frame", "vehicle");

    const auto & pts = j["points"];
    std::size_t raw_n = pts.size();
    if (raw_n == 0) { out_n = 0; return false; }

    std::size_t n = std::min(raw_n, kMaxPoints);

    for (std::size_t i = 0; i < n; ++i) {
      const auto & p = pts[i];
      path_buf_[i].x            = p.value("x", 0.0);
      path_buf_[i].y            = p.value("y", 0.0);
      path_buf_[i].heading      = p.value("heading", 0.0);
      path_buf_[i].curvature    = p.value("curvature", 0.0);
      path_buf_[i].target_speed = p.value("target_speed", 0.0);
    }
    out_n = n; return true;
  } catch (...) {
    out_n = 0; return false;
  }
}

void TrajectoryGeneratorNode::compute_arc_lengths(std::size_t n)
{
  arc_buf_[0] = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    double dx = path_buf_[i].x - path_buf_[i - 1].x;
    double dy = path_buf_[i].y - path_buf_[i - 1].y;
    arc_buf_[i] = std::sqrt(dx * dx + dy * dy);
  }
}

void TrajectoryGeneratorNode::forward_pass(std::size_t n)
{
  vel_fwd_buf_[0] = path_buf_[0].target_speed;
  for (std::size_t i = 1; i < n; ++i) {
    double v_prev = vel_fwd_buf_[i - 1];
    double ds     = arc_buf_[i];
    double v_accel = std::sqrt(v_prev * v_prev + 2.0 * a_max_ * ds);
    // Honor the physical curvature speed limits requested by the Python node!
    vel_fwd_buf_[i] = std::min(path_buf_[i].target_speed, v_accel);
  }
}

void TrajectoryGeneratorNode::backward_pass(std::size_t n)
{
  vel_bwd_buf_[n - 1] = vel_fwd_buf_[n - 1];
  for (std::size_t i = n - 1; i > 0; --i) {
    double v_next = vel_bwd_buf_[i];
    double ds     = arc_buf_[i];
    double v_decel = std::sqrt(v_next * v_next + 2.0 * a_max_ * ds);
    vel_bwd_buf_[i - 1] = std::min(vel_fwd_buf_[i - 1], v_decel);
  }
}

void TrajectoryGeneratorNode::merge_velocity(std::size_t n)
{
  for (std::size_t i = 0; i < n; ++i) {
    traj_buf_[i].x         = path_buf_[i].x;
    traj_buf_[i].y         = path_buf_[i].y;
    traj_buf_[i].heading   = path_buf_[i].heading;
    traj_buf_[i].curvature = path_buf_[i].curvature;
    traj_buf_[i].velocity  = std::max(std::min(vel_fwd_buf_[i], vel_bwd_buf_[i]), 0.0);
    traj_buf_[i].steering_angle = std::atan(wheelbase_ * path_buf_[i].curvature);
  }
}

void TrajectoryGeneratorNode::time_parameterise(std::size_t n)
{
  traj_buf_[0].time_from_start = 0.0;
  traj_buf_[0].acceleration    = 0.0;
  double cumulative_time = 0.0;

  for (std::size_t i = 1; i < n; ++i) {
    double ds = arc_buf_[i];
    double v_avg = std::max(0.5 * (traj_buf_[i - 1].velocity + traj_buf_[i].velocity), epsilon_velocity_);
    double dt = ds / v_avg;
    cumulative_time += dt;
    traj_buf_[i].time_from_start = cumulative_time;
    traj_buf_[i].acceleration = std::clamp((traj_buf_[i].velocity - traj_buf_[i - 1].velocity) / dt, -a_max_, a_max_);
  }
}

void TrajectoryGeneratorNode::publish_trajectory(std::size_t n)
{
  auto msg = std::make_unique<bfmc_msgs::msg::Trajectory>();
  msg->points.reserve(n);
  msg->velocities.reserve(n);
  msg->curvatures.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::Pose p;
    p.position.x = traj_buf_[i].x;
    p.position.y = traj_buf_[i].y;
    p.position.z = 0.0;

    double half_yaw = traj_buf_[i].heading * 0.5;
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = std::sin(half_yaw);
    p.orientation.w = std::cos(half_yaw);

    msg->points.push_back(p);
    msg->velocities.push_back(traj_buf_[i].velocity);
    msg->curvatures.push_back(traj_buf_[i].curvature);
  }

  pub_traj_->publish(std::move(msg));
}

}  // namespace trajectory_generator
