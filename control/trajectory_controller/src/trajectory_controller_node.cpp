#include "trajectory_controller/trajectory_controller_node.hpp"
#include <bfmc_msgs/msg/control_command.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace bfmc_control
{

TrajectoryControllerNode::TrajectoryControllerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("trajectory_controller_node", options)
{
  for (auto & x : x_pred_) x.setZero();
  for (auto & u : u_seq_) u.setZero();
  u_prev_.setZero();
  for (auto & a : A_) a.setIdentity();
  for (auto & b : B_) b.setZero();
  G_.setZero(); H_.setZero(); g_.setZero(); du_.setZero();
  last_traj_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  
  // Initialize our smooth internal speed tracker
  veh_speed_ = 0.0; 
}

TrajectoryControllerNode::CallbackReturn
TrajectoryControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  this->declare_parameter<double>("wheelbase", 0.26);
  this->declare_parameter<double>("a_max", 1.2); 
  this->declare_parameter<double>("delta_max", 0.37);
  this->declare_parameter<double>("v_max", 1.2);
  this->declare_parameter<double>("steering_rate_limit", 1.5);
  this->declare_parameter<int>("max_iterations", 3);
  this->declare_parameter<double>("epsilon_velocity", 0.01);

  this->declare_parameter<double>("w_xy", 1.0); 
  this->declare_parameter<double>("w_psi", 3.0);
  this->declare_parameter<double>("w_v", 2.0);
  this->declare_parameter<double>("w_delta", 0.5);
  this->declare_parameter<double>("w_a", 0.1);
  this->declare_parameter<double>("w_ddelta", 50.0); 
  this->declare_parameter<double>("w_da", 20.0);     

  this->declare_parameter<double>("traj_timeout_s", 0.75);
  // Race-day P4: controller runs at 20 Hz — was 50 Hz. NMPC iteration cost
  // dominated CPU; 20 Hz is enough given vehicle dynamics + 0.05 s NMPC dt.
  this->declare_parameter<double>("control_rate_hz", 20.0);

  this->declare_parameter<double>("min_throttle", 0.05);
  this->declare_parameter<double>("max_throttle", 0.25);
  this->declare_parameter<double>("max_brake", 0.45);
  this->declare_parameter<double>("abs_max_throttle", 0.35);
  this->declare_parameter<double>("abs_max_brake",    1.0);
  this->declare_parameter<double>("goal_radius_m",    0.25);
  this->declare_parameter<double>("goal_v_eps",       0.05);

  L_              = this->get_parameter("wheelbase").as_double();
  a_max_          = this->get_parameter("a_max").as_double();
  delta_max_      = this->get_parameter("delta_max").as_double();
  v_max_          = this->get_parameter("v_max").as_double();
  steer_rate_lim_ = this->get_parameter("steering_rate_limit").as_double();
  max_iter_       = this->get_parameter("max_iterations").as_int();
  eps_vel_        = this->get_parameter("epsilon_velocity").as_double();

  w_xy_     = this->get_parameter("w_xy").as_double();
  w_psi_    = this->get_parameter("w_psi").as_double();
  w_v_      = this->get_parameter("w_v").as_double();
  w_delta_  = this->get_parameter("w_delta").as_double();
  w_a_      = this->get_parameter("w_a").as_double();
  w_ddelta_ = this->get_parameter("w_ddelta").as_double();
  w_da_     = this->get_parameter("w_da").as_double();

  traj_timeout_s_ = this->get_parameter("traj_timeout_s").as_double();
  min_throttle_   = this->get_parameter("min_throttle").as_double();
  max_throttle_   = this->get_parameter("max_throttle").as_double();
  max_brake_      = this->get_parameter("max_brake").as_double();
  abs_max_throttle_ = this->get_parameter("abs_max_throttle").as_double();
  abs_max_brake_    = this->get_parameter("abs_max_brake").as_double();
  goal_radius_m_    = this->get_parameter("goal_radius_m").as_double();
  goal_v_eps_       = this->get_parameter("goal_v_eps").as_double();

  auto be_qos = rclcpp::QoS(1).best_effort().durability_volatile();

  rmw_qos_profile_t cmd_rmw = rmw_qos_profile_default;
  cmd_rmw.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  cmd_rmw.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  cmd_rmw.depth       = 1;
  cmd_rmw.lifespan    = {0, 100'000'000}; 
  auto cmd_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(cmd_rmw));

  sub_traj_ = this->create_subscription<bfmc_msgs::msg::Trajectory>(
    "/planning/trajectory", be_qos, std::bind(&TrajectoryControllerNode::cb_trajectory, this, std::placeholders::_1));

  sub_state_ = this->create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", be_qos, std::bind(&TrajectoryControllerNode::cb_vehicle_state, this, std::placeholders::_1));
    
  sub_estop_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety/emergency_stop", be_qos, std::bind(&TrajectoryControllerNode::cb_estop, this, std::placeholders::_1));

  // Race-day R3: IMU accel-confidence feed into the open-loop speed EMA.
  sub_imu_conf_ = this->create_subscription<std_msgs::msg::Float64>(
    "/fusion/imu_confidence", be_qos,
    [this](std_msgs::msg::Float64::SharedPtr m) {
      const double v = std::isfinite(m->data) ? m->data : 1.0;
      imu_accel_confidence_ = std::clamp(v, 0.0, 1.0);
    });

  pub_cmd_ = this->create_publisher<bfmc_msgs::msg::ControlCommand>("/control/command_raw", cmd_qos);

  return CallbackReturn::SUCCESS;
}

TrajectoryControllerNode::CallbackReturn
TrajectoryControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  pub_cmd_->on_activate();
  double rate_hz = this->get_parameter("control_rate_hz").as_double();
  auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 / rate_hz));
  timer_ = this->create_wall_timer(period, std::bind(&TrajectoryControllerNode::control_tick, this));
  return CallbackReturn::SUCCESS;
}

TrajectoryControllerNode::CallbackReturn
TrajectoryControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  RCLCPP_WARN(this->get_logger(), "CTRL+C DETECTED: Safe Shutdown...");
  publish_neutral();
  pub_cmd_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

TrajectoryControllerNode::CallbackReturn TrajectoryControllerNode::on_cleanup(const rclcpp_lifecycle::State &) { sub_traj_.reset(); sub_state_.reset(); sub_estop_.reset(); pub_cmd_.reset(); return CallbackReturn::SUCCESS; }
TrajectoryControllerNode::CallbackReturn TrajectoryControllerNode::on_shutdown(const rclcpp_lifecycle::State &) { timer_.reset(); return CallbackReturn::SUCCESS; }

void TrajectoryControllerNode::cb_estop(const std_msgs::msg::Bool::SharedPtr msg) { emergency_stop_active_ = msg->data; }

void TrajectoryControllerNode::cb_trajectory(const bfmc_msgs::msg::Trajectory::SharedPtr msg)
{
  if (msg->points.empty()) return;
  ref_count_ = std::min(static_cast<int>(msg->points.size()), MAX_REF);
  if (ref_count_ < 2) return;

  double current_time = 0.0;
  for (int i = 0; i < ref_count_; ++i) {
    ref_raw_[i].x = msg->points[i].position.x;
    ref_raw_[i].y = msg->points[i].position.y;
    double qz = msg->points[i].orientation.z;
    double qw = msg->points[i].orientation.w;
    ref_raw_[i].heading = 2.0 * std::atan2(qz, qw);
    ref_raw_[i].curvature = msg->curvatures[i];
    ref_raw_[i].velocity = msg->velocities[i];

    if (i > 0) {
      double dx = ref_raw_[i].x - ref_raw_[i-1].x;
      double dy = ref_raw_[i].y - ref_raw_[i-1].y;
      double ds = std::sqrt(dx*dx + dy*dy);
      double v_avg = std::max(0.5 * (ref_raw_[i].velocity + ref_raw_[i-1].velocity), 0.01);
      current_time += (ds / v_avg);
    }
    ref_raw_[i].time_from_start = current_time;
  }
  last_traj_stamp_ = this->get_clock()->now();
}

void TrajectoryControllerNode::cb_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  // 🚨 VIBRATION FIX: We explicitly DO NOT trust the EKF speed here.
  // The lack of encoders causes the IMU vibrations to create phantom speeds.
  // veh_speed_ = msg->speed; <-- REMOVED

  if (std::isfinite(msg->pose.position.x) && std::isfinite(msg->pose.position.y)) {
    veh_x_ = msg->pose.position.x;
    veh_y_ = msg->pose.position.y;
  }
  veh_confidence_ = msg->confidence;
  veh_valid_      = true;
}

void TrajectoryControllerNode::control_tick()
{
  if (emergency_stop_active_ || !veh_valid_ || veh_confidence_ <= 0.0) { publish_neutral(); return; }

  auto now = this->get_clock()->now();
  if (ref_count_ < 2 || (now - last_traj_stamp_).seconds() > traj_timeout_s_) { publish_neutral(); return; }

  // Goal-reached latch (race-day P3). Last reference point at zero target
  // velocity + vehicle within goal_radius → strict neutral output, ignore
  // NMPC. Latch sticks until a fresh non-terminal trajectory arrives (i.e.,
  // last ref velocity climbs back above eps).
  {
    const auto & last_ref = ref_raw_[ref_count_ - 1];
    const double dx = veh_x_ - last_ref.x;
    const double dy = veh_y_ - last_ref.y;
    const double d2 = dx * dx + dy * dy;
    const bool   v_zero  = std::abs(last_ref.velocity) <= goal_v_eps_;
    const bool   near    = d2 <= goal_radius_m_ * goal_radius_m_;
    if (v_zero && near) {
      if (!goal_reached_latched_) {
        RCLCPP_INFO(this->get_logger(),
          "goal reached @ (%.2f,%.2f) → strict neutral until trajectory resumes",
          last_ref.x, last_ref.y);
        goal_reached_latched_ = true;
      }
    } else if (!v_zero) {
      // Trajectory has a non-terminal terminal velocity → mission continues.
      goal_reached_latched_ = false;
    }
    if (goal_reached_latched_) { publish_neutral(); return; }
  }

  interpolate_reference();

  // Race-day R3: open-loop speed estimate gated by IMU accel confidence.
  // No wheel encoders, no magnetometer → we can't measure speed directly.
  // The original EMA `0.9*prev + 0.1*target` ignored sensor health and
  // diverged when the planner held a target that the vehicle couldn't
  // achieve (ramp, blocked path, dynamic backoff). New rule:
  //
  //   speed_est = 0.8 * prev + 0.2 * (target_v * imu_accel_conf)
  //
  // imu_accel_conf ∈ [0,1] from /fusion/imu_confidence. When IMU degrades
  // the estimate naturally decays toward 0.8 * prev (i.e. ride the previous
  // estimate, don't follow the planner blindly). When IMU is healthy the
  // estimate tracks the planner target with the same 5-step time constant
  // as before.
  double target_v = ref_interp_[0].velocity;
  veh_speed_ = 0.8 * veh_speed_ + 0.2 * (target_v * imu_accel_confidence_);

  warm_start();

  for (int iter = 0; iter < max_iter_; ++iter) {
    forward_simulate();     
    compute_jacobians();    
    build_qp();             
    solve_qp();             
    for (int k = 0; k < NH; ++k) {
      double du_steer = std::clamp(du_(k * NU + 0), -0.08, 0.08);
      double du_accel = std::clamp(du_(k * NU + 1), -0.5, 0.5);
      u_seq_[k](0) += du_steer;
      u_seq_[k](1) += du_accel;
    }
    project_constraints();  
  }

  publish_command();
  u_prev_ = u_seq_[0];
}

void TrajectoryControllerNode::interpolate_reference()
{
  for (int k = 0; k <= NH; ++k) {
    const double t = k * DT;
    int idx = 0;
    for (int i = 0; i < ref_count_ - 1; ++i) {
      if (ref_raw_[i + 1].time_from_start >= t) { idx = i; break; }
      idx = i;
    }

    const int idx1 = std::min(idx + 1, ref_count_ - 1);
    const double t0 = ref_raw_[idx].time_from_start;
    const double t1 = ref_raw_[idx1].time_from_start;

    double alpha = 0.0;
    if (t1 > t0 + 1e-9) alpha = std::clamp((t - t0) / (t1 - t0), 0.0, 1.0);

    const auto & p0 = ref_raw_[idx];
    const auto & p1 = ref_raw_[idx1];

    ref_interp_[k].x         = p0.x + alpha * (p1.x - p0.x);
    ref_interp_[k].y         = p0.y + alpha * (p1.y - p0.y);
    ref_interp_[k].heading   = p0.heading + alpha * wrap_angle(p1.heading - p0.heading);
    ref_interp_[k].velocity  = p0.velocity + alpha * (p1.velocity - p0.velocity);
    ref_interp_[k].curvature = p0.curvature + alpha * (p1.curvature - p0.curvature);
  }
}

void TrajectoryControllerNode::warm_start() { for (int k = 0; k < NH - 1; ++k) { u_seq_[k] = u_seq_[k + 1]; } u_seq_[NH - 1] = u_seq_[NH - 2]; }

StateVec TrajectoryControllerNode::bicycle_predict(const StateVec & x, const CtrlVec & u) const
{
  const double X   = x(0); const double Y   = x(1); const double psi = x(2); const double v   = x(3);
  const double delta = u(0); const double a      = u(1);

  StateVec xn;
  xn(0) = X + v * std::cos(psi) * DT;
  xn(1) = Y + v * std::sin(psi) * DT;
  xn(2) = psi + (v / L_) * std::tan(delta) * DT;
  
  const double kv = 0.8; 
  xn(3) = v + (a - kv * v) * DT;
  
  xn(2) = wrap_angle(xn(2));
  xn(3) = std::clamp(xn(3), 0.0, v_max_);
  return xn;
}

StateJac TrajectoryControllerNode::bicycle_jacobian_A(const StateVec & x, const CtrlVec & u) const
{
  const double psi = x(2); const double v = x(3); const double delta = u(0);
  const double cp = std::cos(psi); const double sp = std::sin(psi); const double td = std::tan(delta);

  StateJac A; A.setIdentity();
  A(0, 2) = -v * sp * DT; A(0, 3) =  cp * DT;
  A(1, 2) =  v * cp * DT; A(1, 3) =  sp * DT;
  A(2, 3) =  td / L_ * DT;
  A(3, 3) =  1.0 - 0.8 * DT; 
  return A;
}

InputJac TrajectoryControllerNode::bicycle_jacobian_B(const StateVec & x, const CtrlVec & u) const
{
  const double v = x(3); const double delta = u(0);
  const double cd = std::cos(delta); const double cd2 = std::max(cd * cd, 1e-6);

  InputJac B; B.setZero();
  B(2, 0) = v * DT / (L_ * cd2); B(3, 1) = DT;
  return B;
}

void TrajectoryControllerNode::forward_simulate()
{
  x_pred_[0].setZero(); x_pred_[0](3) = veh_speed_;
  for (int k = 0; k < NH; ++k) {
    u_seq_[k](0) = std::clamp(u_seq_[k](0), -0.32, 0.32); 
    x_pred_[k + 1] = bicycle_predict(x_pred_[k], u_seq_[k]);
  }
}

void TrajectoryControllerNode::compute_jacobians() { for (int k = 0; k < NH; ++k) { A_[k] = bicycle_jacobian_A(x_pred_[k], u_seq_[k]); B_[k] = bicycle_jacobian_B(x_pred_[k], u_seq_[k]); } }

void TrajectoryControllerNode::build_qp()
{
  H_.setZero(); g_.setZero(); G_.setZero();

  for (int j = 0; j < NH; ++j) {
    G_.block<NX, NU>(j * NX, j * NU) = B_[j];
    for (int i = j + 1; i < NH; ++i) G_.block<NX, NU>(i * NX, j * NU).noalias() = A_[i] * G_.block<NX, NU>((i - 1) * NX, j * NU);
  }

  for (int k = 0; k < NH; ++k) {
    StateVec e_k;
    e_k(0) = x_pred_[k + 1](0) - ref_interp_[k + 1].x;
    e_k(1) = x_pred_[k + 1](1) - ref_interp_[k + 1].y;
    e_k(2) = wrap_angle(x_pred_[k + 1](2) - ref_interp_[k + 1].heading);
    e_k(3) = x_pred_[k + 1](3) - ref_interp_[k + 1].velocity;

    double yaw = ref_interp_[k + 1].heading;
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    double w_lon = w_xy_ * 0.1; 
    double w_lat = w_xy_ * 2.0; 

    Eigen::Matrix4d Q; Q.setZero();
    Q(0,0) = w_lon * cy * cy + w_lat * sy * sy;
    Q(0,1) = (w_lon - w_lat) * cy * sy;
    Q(1,0) = Q(0,1);
    Q(1,1) = w_lon * sy * sy + w_lat * cy * cy;
    Q(2,2) = w_psi_;
    Q(3,3) = w_v_;

    Eigen::Matrix<double, NX, NZ> QGk = Q * G_.block<NX, NZ>(k * NX, 0);

    H_.noalias() += G_.block<NX, NZ>(k * NX, 0).transpose() * QGk;
    StateVec Qe = Q * e_k;
    g_.noalias() += G_.block<NX, NZ>(k * NX, 0).transpose() * Qe;
  }

  for (int k = 0; k < NH; ++k) {
    const int idx = k * NU;
    H_(idx,     idx)     += w_delta_;
    H_(idx + 1, idx + 1) += w_a_;
    g_(idx)     += w_delta_ * u_seq_[k](0);
    g_(idx + 1) += w_a_     * u_seq_[k](1);
  }

  for (int k = 0; k < NH; ++k) {
    const int idx = k * NU;
    const CtrlVec u_km1 = (k == 0) ? u_prev_ : u_seq_[k - 1];
    const double dd = u_seq_[k](0) - u_km1(0);
    const double da = u_seq_[k](1) - u_km1(1);

    H_(idx,     idx)     += w_ddelta_;
    H_(idx + 1, idx + 1) += w_da_;

    if (k > 0) {
      const int pidx = (k - 1) * NU;
      H_(pidx,     pidx)     += w_ddelta_;
      H_(pidx + 1, pidx + 1) += w_da_;
      H_(idx,  pidx)     -= w_ddelta_;
      H_(pidx, idx)      -= w_ddelta_;
      H_(idx + 1, pidx + 1) -= w_da_;
      H_(pidx + 1, idx + 1) -= w_da_;
    }

    g_(idx)     += w_ddelta_ * dd;
    g_(idx + 1) += w_da_     * da;

    if (k > 0) {
      const int pidx = (k - 1) * NU;
      g_(pidx)     -= w_ddelta_ * dd;
      g_(pidx + 1) -= w_da_     * da;
    }
  }
}

void TrajectoryControllerNode::solve_qp()
{
  for (int i = 0; i < NZ; ++i) H_(i, i) += 1e-6;
  Eigen::LLT<QpMat> llt(H_);
  if (llt.info() == Eigen::Success) du_ = llt.solve(-g_);
  else du_ = -0.01 * g_;
}

void TrajectoryControllerNode::project_constraints()
{
  double v = veh_speed_;
  for (int k = 0; k < NH; ++k) {
    u_seq_[k](0) = std::clamp(u_seq_[k](0), -delta_max_, delta_max_);

    const double delta_prev = (k == 0) ? u_prev_(0) : u_seq_[k - 1](0);
    const double max_change = steer_rate_lim_ * DT;
    double ddelta = u_seq_[k](0) - delta_prev;
    ddelta = std::clamp(ddelta, -max_change, max_change);
    u_seq_[k](0) = delta_prev + ddelta;
    u_seq_[k](0) = std::clamp(u_seq_[k](0), -delta_max_, delta_max_);

    u_seq_[k](1) = std::clamp(u_seq_[k](1), -a_max_, a_max_);

    double v_next = v + u_seq_[k](1) * DT;
    if (v_next < 0.0) { u_seq_[k](1) = std::max(-v / DT, -a_max_); v_next = 0.0; }
    if (v_next > v_max_) { u_seq_[k](1) = std::min((v_max_ - v) / DT, a_max_); v_next = v_max_; }
    v = v_next;
  }
}

void TrajectoryControllerNode::publish_command()
{
  const double delta = u_seq_[0](0);
  const double a_ref = u_seq_[0](1);
  (void)a_ref;  // NMPC accel is informational here; throttle is feed-forward below.

  double throttle = 0.0;
  double brake    = 0.0;

  // Bug-1 — intent-driven forward / neutral decision.
  //
  // Previous logic gated on `veh_speed_ > 0.05` (the open-loop EMA), but the
  // EMA asymptotes to target_v and never strictly EXCEEDS it. So when the
  // FSM commanded target_v = 0.05 m/s (FOLLOW tailing_min_speed), the EMA
  // converged to 0.05, the strict-greater test failed, and the controller
  // fell to NEUTRAL — the car froze at every false-positive FOLLOW trigger
  // with zero steering. Symptoms in the bench log: car stops at "every
  // sign it sees" + no steering.
  //
  // New logic: forward/brake/neutral are determined by what the PLANNER
  // asks for (target_v), not by our estimate of what the car is doing.
  //
  //   A) target_v > kTargetMoveEps                 → FORWARD throttle.
  //      Throttle scales with target_v, clamped to [min_throttle_, max_throttle_].
  //      As long as the planner asks for ≥3 cm/s, we drive.
  //   B) target_v ~ 0  AND  veh_speed_ > kStoppedThresh
  //                                                → BRAKE (gentle stop).
  //   C) target_v ~ 0  AND  veh_speed_ ≤ kStoppedThresh
  //                                                → STRICT NEUTRAL.
  //                                                  STM32 sees NEUTRAL_PWM.
  constexpr double kStoppedThresh  = 0.02;   // m/s — "we have stopped"
  constexpr double kTargetMoveEps  = 0.03;   // m/s — planner asks us to move
  constexpr double kTargetStopEps  = 0.03;   // m/s — planner asks us to stop

  const double target_v = (ref_count_ > 0) ? ref_interp_[0].velocity : 0.0;

  if (target_v > kTargetMoveEps) {
    // (A) Planner wants us moving — feedforward throttle on target_v.
    // norm_v ∈ [0,1] mapped to [min_throttle_, max_throttle_] linearly.
    const double norm_v = std::clamp(target_v / v_max_, 0.0, 1.0);
    throttle = min_throttle_ + norm_v * (max_throttle_ - min_throttle_);
  } else if (target_v <= kTargetStopEps && veh_speed_ > kStoppedThresh) {
    // (B) Planner wants stop, we're still moving — gentle brake.
    brake = 0.15;
  } else {
    // (C) Stopped + planner says stop — STRICT NEUTRAL.
    throttle = 0.0;
    brake    = 0.0;
  }

  // Race-day P3: hard clamp at publish time. throttle ∈ [0, abs_max_throttle_],
  // brake ∈ [0, abs_max_brake_]. The lower bound of 0 on throttle physically
  // forbids reverse motion — no NMPC solver output, no transient, no warm-start
  // overshoot can ever drive the wheels backward.
  throttle = std::clamp(throttle, 0.0, abs_max_throttle_);
  brake    = std::clamp(brake,    0.0, abs_max_brake_);

  bfmc_msgs::msg::ControlCommand cmd_msg;
  cmd_msg.steering_angle = static_cast<float>(delta);
  cmd_msg.throttle       = static_cast<float>(throttle);
  cmd_msg.brake          = static_cast<float>(brake);
  pub_cmd_->publish(cmd_msg);
}

void TrajectoryControllerNode::publish_neutral()
{
  bfmc_msgs::msg::ControlCommand cmd_msg;
  cmd_msg.steering_angle = 0.0; cmd_msg.throttle = 0.0; cmd_msg.brake = 0.0; 
  pub_cmd_->publish(cmd_msg);
}

double TrajectoryControllerNode::wrap_angle(double a) { while (a > M_PI) a -= 2.0 * M_PI; while (a < -M_PI) a += 2.0 * M_PI; return a; }

}  // namespace bfmc_control
