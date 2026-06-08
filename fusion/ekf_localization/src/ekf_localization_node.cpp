#include "ekf_localization/ekf_localization_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>           // tf2::getYaw — preferred over getRPY when only yaw is needed
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

namespace bfmc_fusion
{

EkfLocalizationNode::EkfLocalizationNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ekf_localization_node", options),
  last_imu_time_(0, 0, RCL_ROS_TIME),
  last_imu_stamp_seen_(0, 0, RCL_ROS_TIME),
  prev_loc_stamp_(0, 0, RCL_ROS_TIME),
  last_loc_stamp_(0, 0, RCL_ROS_TIME)
{
  x_.setZero();
  P_.setIdentity();
  P_ *= 0.1;

  Q_.setZero();
  // Process noise: [x, y, yaw, yaw_bias]
  Q_.diagonal() << 0.01, 0.01, 0.05, 0.001;

  // R_yaw_ holds the BASE measurement variance; it is scaled per-tick by
  // imu_confidence in the yaw update (R_eff = R_YAW_BASE / max(conf, MIN)).
  R_yaw_.setIdentity();
  R_yaw_(0, 0) = R_YAW_BASE;

  R_xy_.setZero();
  R_xy_(0, 0) = LOC_STD * LOC_STD;
  R_xy_(1, 1) = LOC_STD * LOC_STD;

  F_.setIdentity();
  I_.setIdentity();

  H_yaw_.setZero();
  H_yaw_(0, 2) = 1.0;

  H_xy_.setZero();
  H_xy_(0, 0) = 1.0;
  H_xy_(1, 1) = 1.0;
}

CallbackReturn EkfLocalizationNode::on_configure(const rclcpp_lifecycle::State &)
{
  auto imu_qos   = rclcpp::QoS(5).best_effort();
  auto state_qos = rclcpp::QoS(10).reliable();
  auto drift_qos = rclcpp::QoS(5).best_effort();
  auto loc_qos   = rclcpp::QoS(5).reliable();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_filtered", imu_qos,
    std::bind(&EkfLocalizationNode::imu_callback, this, std::placeholders::_1));

  drift_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/fusion/yaw_bias_correction", drift_qos,
    std::bind(&EkfLocalizationNode::correction_callback, this, std::placeholders::_1));

  loc_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/localization/raw_position", loc_qos,
    std::bind(&EkfLocalizationNode::localization_callback, this, std::placeholders::_1));

  state_pub_ = create_publisher<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state_raw", state_qos);

  // ---- Phase 2.5: adaptive IMU diagnostics --------------------------------
  enable_debug_topics_ = declare_parameter<bool>("enable_debug_topics", enable_debug_topics_);

  auto dbg_qos = rclcpp::QoS(5).best_effort();
  // /fusion/imu_confidence is always on — safety_supervisor and health_monitor
  // both consume it. The /fusion/debug/* trio mirror the diagnostic style of
  // drift_estimator (gated, lazy-created).
  imu_confidence_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/fusion/imu_confidence", dbg_qos);

  if (enable_debug_topics_) {
    debug_yaw_measurement_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/fusion/debug/yaw_measurement", dbg_qos);
    debug_yaw_bias_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/fusion/debug/yaw_bias", dbg_qos);
    debug_imu_confidence_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/fusion/debug/imu_confidence", dbg_qos);
    RCLCPP_INFO(get_logger(),
      "ekf_localization: IMU debug topics ENABLED "
      "(/fusion/debug/{yaw_measurement,yaw_bias,imu_confidence})");
  } else {
    RCLCPP_INFO(get_logger(),
      "ekf_localization: IMU debug topics disabled "
      "(only /fusion/imu_confidence will be published)");
  }

  // ---- Phase 2.6: CSV logger ----------------------------------------------
  enable_csv_logging_ = declare_parameter<bool>("enable_csv_logging", enable_csv_logging_);
  csv_log_dir_        = declare_parameter<std::string>("csv_log_dir",   csv_log_dir_);

  if (enable_csv_logging_) {
    if (open_csv_logs_()) {
      RCLCPP_INFO(get_logger(),
        "ekf_localization: CSV logging ENABLED → %s/{imu_confidence,"
        "yaw_measurement,yaw_bias,vehicle_state}.csv (flush every %lu ticks)",
        csv_log_dir_.c_str(),
        static_cast<unsigned long>(CSV_FLUSH_EVERY));
    } else {
      // open_csv_logs_ already logged the reason; disable to avoid noisy
      // per-tick warnings.
      enable_csv_logging_ = false;
      RCLCPP_ERROR(get_logger(),
        "ekf_localization: CSV logging DISABLED (could not open files in %s). "
        "Set csv_log_dir to a writable location and re-launch to enable.",
        csv_log_dir_.c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(),
      "ekf_localization: CSV logging disabled (set enable_csv_logging:=true to enable)");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EkfLocalizationNode::on_activate(const rclcpp_lifecycle::State &)
{
  state_pub_->on_activate();
  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&EkfLocalizationNode::timer_step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn EkfLocalizationNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn EkfLocalizationNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  imu_sub_.reset();
  drift_sub_.reset();
  loc_sub_.reset();
  state_pub_.reset();
  imu_confidence_pub_.reset();
  debug_yaw_measurement_pub_.reset();
  debug_yaw_bias_pub_.reset();
  debug_imu_confidence_pub_.reset();
  close_csv_logs_();
  return CallbackReturn::SUCCESS;
}

void EkfLocalizationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = *msg;
  imu_received_ = true;
  last_imu_stamp_seen_ = now();
}

void EkfLocalizationNode::correction_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_correction_ = msg->data;
}

void EkfLocalizationNode::localization_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  pending_loc_x_  = msg->point.x;
  pending_loc_y_  = msg->point.y;
  loc_pending_    = true;
  loc_received_   = true;
  last_loc_stamp_ = now();

  // v = sqrt(dx^2 + dy^2) / dt, low-passed per spec.
  // Always non-negative (loc deltas are scalar distances).
  if (loc_have_prev_) {
    const double dt = (last_loc_stamp_ - prev_loc_stamp_).seconds();
    if (dt > 0.01 && dt < 5.0) {
      const double dx = pending_loc_x_ - prev_loc_x_;
      const double dy = pending_loc_y_ - prev_loc_y_;
      const double v_inst = std::sqrt(dx * dx + dy * dy) / dt;
      v_loc_filtered_ = V_LOC_ALPHA * v_inst + (1.0 - V_LOC_ALPHA) * v_loc_filtered_;
      v_loc_filtered_ = std::clamp(v_loc_filtered_, 0.0, MAX_VEL);
    }
  }
  prev_loc_x_     = pending_loc_x_;
  prev_loc_y_     = pending_loc_y_;
  prev_loc_stamp_ = last_loc_stamp_;
  loc_have_prev_  = true;
}

double EkfLocalizationNode::normalize_yaw(double yaw)
{
  while (yaw >  M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

void EkfLocalizationNode::timer_step()
{
  if (!imu_received_) return;

  const rclcpp::Time imu_time(latest_imu_.header.stamp);
  const rclcpp::Time tnow = now();
  if ((tnow - imu_time).seconds() > IMU_TIMEOUT) return;
  if (!std::isfinite(latest_imu_.angular_velocity.z) ||
      !std::isfinite(latest_imu_.linear_acceleration.x))
    return;

  if (last_imu_time_.nanoseconds() == 0) {
    last_imu_time_ = imu_time;
    return;
  }

  double dt = (imu_time - last_imu_time_).seconds();
  if (dt <= 0.0) return;
  if (dt > MAX_DT) dt = 0.01;
  last_imu_time_ = imu_time;

  // ====== Phase 2.5/2.6: IMU orientation sanity gates =======================
  // The STM32 already runs onboard sensor fusion (gyro + accel + mag) and
  // publishes a fused orientation quaternion with valid covariances on
  // /microros/imu_raw. We use that as a MEASUREMENT — we do not estimate
  // orientation a second time here. Phase 2.6 additionally removes the
  // accelerometer-derived forward-speed path: the body-frame accel is no
  // longer integrated for vehicle speed, so the only consumer of the
  // quaternion below is the yaw measurement update.
  const double qx = latest_imu_.orientation.x;
  const double qy = latest_imu_.orientation.y;
  const double qz = latest_imu_.orientation.z;
  const double qw = latest_imu_.orientation.w;

  const bool quat_finite =
    std::isfinite(qx) && std::isfinite(qy) &&
    std::isfinite(qz) && std::isfinite(qw);

  const double quat_norm =
    quat_finite ? std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw) : 0.0;

  const double orient_cov_yaw = latest_imu_.orientation_covariance[8];
  const bool orient_cov_ok =
    std::isfinite(orient_cov_yaw) && orient_cov_yaw >= 0.0 &&
    orient_cov_yaw <= ORIENT_COV_MAX;

  const bool quat_norm_ok =
    quat_norm >= QUAT_NORM_MIN && quat_norm <= QUAT_NORM_MAX;

  const bool orientation_valid = quat_finite && quat_norm_ok && orient_cov_ok;

  // ---- Adaptive IMU confidence (Phase 2.7 trace-based) ---------------------
  // Yaw confidence is dominated by the fused quaternion / magnetometer / gyro;
  // accel contributes only marginally now that we never integrate it.
  //   imu_confidence = exp(-(0.6*σ_orient + 0.3*σ_gyro + 0.1*σ_accel))
  // where each σ_X is the square root of the TRACE of its 3x3 covariance —
  // i.e. cov[0] + cov[4] + cov[8] — so all three axes contribute. Only the
  // diagonal terms are meaningful; off-diagonals (cross-axis correlations)
  // are deliberately ignored. Healthy STM32 covariances on all three axes:
  //   orient_diag ≈ [0.001, 0.001, 0.01]   → σ ≈ √0.012 ≈ 0.110
  //   gyro_diag   ≈ 2.19e-6 (isotropic)    → σ ≈ √6.57e-6 ≈ 2.56e-3
  //   accel_diag  ≈ 0.004  (isotropic)     → σ ≈ √0.012 ≈ 0.110
  //   weighted_σ ≈ 0.6·0.110 + 0.3·2.56e-3 + 0.1·0.110 ≈ 0.078
  //   confidence ≈ exp(-0.078) ≈ 0.925.
  // The hard orientation-yaw reject (orient_cov_yaw > 0.5) is separate and
  // unchanged — see orient_cov_ok above.
  auto trace_sigma = [](const std::array<double,9> & cov) -> double {
    const double t = std::max(0.0, cov[0]) + std::max(0.0, cov[4]) + std::max(0.0, cov[8]);
    return std::sqrt(t);
  };
  const double sigma_orient = trace_sigma(latest_imu_.orientation_covariance);
  const double sigma_gyro   = trace_sigma(latest_imu_.angular_velocity_covariance);
  const double sigma_accel  = trace_sigma(latest_imu_.linear_acceleration_covariance);
  const double weighted_sigma =
      W_CONF_ORIENT * sigma_orient +
      W_CONF_GYRO   * sigma_gyro   +
      W_CONF_ACCEL  * sigma_accel;
  imu_confidence_ = std::isfinite(weighted_sigma)
    ? std::clamp(std::exp(-weighted_sigma), 0.0, 1.0)
    : 0.0;

  // ====== STEP 1: Speed source (Phase 2.6 — localization-only) ==============
  // We do NOT integrate accel for speed. On a BFMC vehicle, the noise floor
  // of the BNO085 accelerometer + chassis vibration + double-integration
  // drift produce false speed estimates that a Kalman filter happily locks
  // onto. The only trusted source is the temporal derivative of successive
  // localization fixes (v_loc_filtered, low-passed in localization_callback).
  // When localization is stale we report zero — we do NOT extrapolate motion
  // from inertial data.
  const double loc_age = loc_received_
    ? (tnow - last_loc_stamp_).seconds()
    : std::numeric_limits<double>::infinity();
  const bool loc_fresh = (loc_age < LOC_TIMEOUT);

  v_blended_ = loc_fresh ? v_loc_filtered_ : 0.0;
  v_blended_ = std::clamp(v_blended_, 0.0, MAX_VEL);

  // ====== STEP 3: PREDICT ======
  const double gyro_z = latest_imu_.angular_velocity.z;
  const double yaw_rate = gyro_z - x_(3);

  // Drift bias correction integrated over this dt.
  x_(3) += latest_correction_ * dt;
  latest_correction_ = 0.0;

  const double yaw_prev = x_(2);

  // Dead-reckon position with v_blended_ along previous yaw.
  x_(0) += v_blended_ * std::cos(yaw_prev) * dt;
  x_(1) += v_blended_ * std::sin(yaw_prev) * dt;
  x_(2) += yaw_rate * dt;
  x_(2)  = normalize_yaw(x_(2));
  // yaw_bias is a random walk; covariance carries the uncertainty.

  // Jacobian F = ∂f/∂x for state [x, y, yaw, b]:
  F_.setIdentity();
  F_(0, 2) = -v_blended_ * std::sin(yaw_prev) * dt;
  F_(1, 2) =  v_blended_ * std::cos(yaw_prev) * dt;
  F_(2, 3) = -dt;

  P_ = F_ * P_ * F_.transpose() + Q_ * dt;
  P_ = 0.5 * (P_ + P_.transpose());

  // ====== STEP 4: UPDATE yaw from BNO orientation (Phase 2.5) ===============
  // Measurement-only path. STM32 already performs onboard sensor fusion;
  // avoid estimating orientation twice — the yaw measurement only ever
  // CORRECTS the gyro-integrated prediction via a Kalman update, never
  // replaces it directly.
  double yaw_meas = 0.0;
  bool yaw_update_applied = false;

  if (orientation_valid) {
    // tf2::getYaw avoids the roll/pitch decomposition cost of getRPY.
    tf2::Quaternion q_meas(qx, qy, qz, qw);
    q_meas.normalize();
    yaw_meas = tf2::getYaw(q_meas);
    const double innov_yaw = normalize_yaw(yaw_meas - x_(2));

    if (std::abs(innov_yaw) < YAW_GATE) {
      // Adaptive R: scale the BASE variance up when imu_confidence is low.
      // R_eff = R_YAW_BASE / max(imu_confidence, IMU_CONF_MIN)
      // ⇒ noisy IMU is automatically trusted less. IMU_CONF_MIN floors the
      //   divisor so a near-zero confidence still produces a finite R.
      Eigen::Matrix<double, 1, 1> R_yaw_adaptive;
      R_yaw_adaptive(0, 0) =
        R_YAW_BASE / std::max(imu_confidence_, IMU_CONF_MIN);

      const auto S = H_yaw_ * P_ * H_yaw_.transpose() + R_yaw_adaptive;
      const auto K = P_ * H_yaw_.transpose() * S.inverse();
      x_ += K * innov_yaw;
      x_(2) = normalize_yaw(x_(2));
      P_ = (I_ - K * H_yaw_) * P_;
      P_ = 0.5 * (P_ + P_.transpose());
      yaw_update_applied = true;
    }
  }

  // ====== STEP 5: UPDATE (x, y) from localization (when a fix is pending) ======
  if (loc_pending_) {
    Eigen::Matrix<double, 2, 1> innov_xy;
    innov_xy(0) = pending_loc_x_ - x_(0);
    innov_xy(1) = pending_loc_y_ - x_(1);

    const auto S2 = H_xy_ * P_ * H_xy_.transpose() + R_xy_;
    const auto K2 = P_ * H_xy_.transpose() * S2.inverse();
    x_ += K2 * innov_xy;
    x_(2) = normalize_yaw(x_(2));
    P_ = (I_ - K2 * H_xy_) * P_;
    P_ = 0.5 * (P_ + P_.transpose());
    loc_pending_ = false;
  }

  P_ += 1e-9 * I_;

  // ====== STEP 6: Smooth health-based confidence ======
  // IMU health: linear decay from 1.0 at age 0 to 0.0 at age IMU_HEALTH_TAU.
  const double imu_age = (tnow - last_imu_stamp_seen_).seconds();
  double imu_health = 1.0 - (imu_age / IMU_HEALTH_TAU);
  imu_health = std::clamp(imu_health, 0.0, 1.0);

  // Localization health: same shape, longer tail.
  double loc_health = loc_received_ ? (1.0 - loc_age / LOC_TIMEOUT) : 0.0;
  loc_health = std::clamp(loc_health, 0.0, 1.0);

  // Covariance health: penalize large position uncertainty.
  const double cov_pos = P_(0, 0) + P_(1, 1);
  double cov_health = 1.0 - cov_pos / COV_POS_MAX;
  cov_health = std::clamp(cov_health, 0.0, 1.0);

  const double confidence = std::clamp(
    W_IMU_HEALTH * imu_health + W_LOC_HEALTH * loc_health + W_COV_HEALTH * cov_health,
    0.0, 1.0);

  // ====== STEP 7: OUTPUT ======
  state_msg_.pose.position.x = x_(0);
  state_msg_.pose.position.y = x_(1);
  state_msg_.pose.position.z = 0.0;

  tf2::Quaternion q_out;
  q_out.setRPY(0.0, 0.0, x_(2));
  state_msg_.pose.orientation.x = q_out.x();
  state_msg_.pose.orientation.y = q_out.y();
  state_msg_.pose.orientation.z = q_out.z();
  state_msg_.pose.orientation.w = q_out.w();

  state_msg_.twist.linear.x  = v_blended_;
  state_msg_.twist.linear.y  = 0.0;
  state_msg_.twist.linear.z  = 0.0;
  state_msg_.twist.angular.x = 0.0;
  state_msg_.twist.angular.y = 0.0;
  state_msg_.twist.angular.z = yaw_rate;

  state_msg_.yaw        = static_cast<float>(x_(2));
  state_msg_.yaw_rate   = static_cast<float>(yaw_rate);
  state_msg_.speed      = static_cast<float>(v_blended_);
  state_msg_.confidence = static_cast<float>(confidence);

  state_pub_->publish(state_msg_);

  // ====== Phase 2.5: diagnostics ============================================
  // /fusion/imu_confidence is always on so the safety supervisor and
  // health_monitor can react to a degrading IMU even when debug topics are
  // disabled in production.
  if (imu_confidence_pub_) {
    std_msgs::msg::Float64 m;
    m.data = imu_confidence_;
    imu_confidence_pub_->publish(m);
  }

  // Debug-namespace mirror. yaw_measurement is only published when the IMU
  // sample passed all gates this tick — a gap on echo means the IMU was
  // rejected, which is itself useful signal.
  if (enable_debug_topics_) {
    if (yaw_update_applied && debug_yaw_measurement_pub_) {
      std_msgs::msg::Float64 m;
      m.data = yaw_meas;
      debug_yaw_measurement_pub_->publish(m);
    }
    if (debug_yaw_bias_pub_) {
      std_msgs::msg::Float64 m;
      m.data = x_(3);   // EKF state element [3] = yaw_bias (rad/s)
      debug_yaw_bias_pub_->publish(m);
    }
    if (debug_imu_confidence_pub_) {
      std_msgs::msg::Float64 m;
      m.data = imu_confidence_;
      debug_imu_confidence_pub_->publish(m);
    }
  }

  // ====== Phase 2.6: per-tick CSV row (one snapshot across the 4 files) ===
  if (enable_csv_logging_) {
    write_csv_row_(
      tnow.seconds(),
      yaw_meas,
      x_(3),
      imu_confidence_,
      x_(2),
      v_blended_,
      yaw_update_applied);
  }
}

// ============================================================================
// CSV logger (Phase 2.6)
// ============================================================================
bool EkfLocalizationNode::open_csv_logs_()
{
  // Materialise the target directory; std::filesystem swallows the "already
  // exists" case and only throws on hard errors (EACCES, ENOSPC, …).
  std::error_code ec;
  std::filesystem::create_directories(csv_log_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(get_logger(),
      "ekf_localization: cannot create CSV log dir '%s': %s",
      csv_log_dir_.c_str(), ec.message().c_str());
    return false;
  }

  auto open_one = [&](std::ofstream & f, const std::string & name,
                      const std::string & header) -> bool {
    const std::string path =
      (std::filesystem::path(csv_log_dir_) / name).string();
    // Append-mode + ate: open at end so we can detect "empty file ⇒ first
    // open ⇒ needs a header". Existing files are preserved across launches.
    f.open(path, std::ios::out | std::ios::app | std::ios::ate);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(),
        "ekf_localization: cannot open CSV file '%s'", path.c_str());
      return false;
    }
    if (f.tellp() == std::streampos(0)) {
      f << header << "\n";
    }
    return true;
  };

  // Each file holds (timestamp, <its column>). One row per timer tick.
  if (!open_one(csv_imu_confidence_, "imu_confidence.csv",
                "timestamp,imu_confidence")) return false;
  if (!open_one(csv_yaw_measurement_, "yaw_measurement.csv",
                "timestamp,yaw_measurement,valid")) return false;
  if (!open_one(csv_yaw_bias_, "yaw_bias.csv",
                "timestamp,yaw_bias")) return false;
  if (!open_one(csv_vehicle_state_, "vehicle_state.csv",
                "timestamp,vehicle_yaw,vehicle_speed")) return false;
  return true;
}

void EkfLocalizationNode::close_csv_logs_()
{
  // Best-effort flush + close. ofstream's destructor would do this anyway,
  // but doing it explicitly on cleanup gives us a sync point for race-day
  // post-mortems even if the process is later force-killed.
  for (auto * f : {&csv_imu_confidence_, &csv_yaw_measurement_,
                   &csv_yaw_bias_,        &csv_vehicle_state_})
  {
    if (f->is_open()) { f->flush(); f->close(); }
  }
}

void EkfLocalizationNode::write_csv_row_(double t,
                                         double yaw_measurement,
                                         double yaw_bias,
                                         double imu_conf,
                                         double veh_yaw,
                                         double veh_speed,
                                         bool   yaw_meas_valid)
{
  // ofstream writes are line-buffered by default for typical sizes; we
  // explicitly flush() every CSV_FLUSH_EVERY ticks so a crash loses at most
  // one second of data at 100 Hz.
  if (csv_imu_confidence_.is_open()) {
    csv_imu_confidence_ << std::fixed << std::setprecision(6)
      << t << "," << imu_conf << "\n";
  }
  if (csv_yaw_measurement_.is_open()) {
    csv_yaw_measurement_ << std::fixed << std::setprecision(6) << t << ",";
    if (yaw_meas_valid) {
      csv_yaw_measurement_ << yaw_measurement;
    } else {
      csv_yaw_measurement_ << "nan";
    }
    csv_yaw_measurement_ << "," << (yaw_meas_valid ? "1" : "0") << "\n";
  }
  if (csv_yaw_bias_.is_open()) {
    csv_yaw_bias_ << std::fixed << std::setprecision(6)
      << t << "," << yaw_bias << "\n";
  }
  if (csv_vehicle_state_.is_open()) {
    csv_vehicle_state_ << std::fixed << std::setprecision(6)
      << t << "," << veh_yaw << "," << veh_speed << "\n";
  }

  if (++csv_tick_counter_ >= CSV_FLUSH_EVERY) {
    csv_imu_confidence_.flush();
    csv_yaw_measurement_.flush();
    csv_yaw_bias_.flush();
    csv_vehicle_state_.flush();
    csv_tick_counter_ = 0;
  }
}

}  // namespace bfmc_fusion
