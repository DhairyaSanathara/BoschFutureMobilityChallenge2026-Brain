#ifndef EKF_LOCALIZATION__EKF_LOCALIZATION_NODE_HPP_
#define EKF_LOCALIZATION__EKF_LOCALIZATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <string>

namespace bfmc_fusion
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// EKF over [x, y, yaw, yaw_bias].
//
// Phase 2.6: ACCELEROMETER-DERIVED VEHICLE SPEED REMOVED.
// We do not integrate linear_acceleration. On a BFMC-scale vehicle, sensor
// noise + chassis vibration + double-integration drift make accel-derived
// speed worse than useless — a Kalman filter will happily lock onto a few
// cm/s of drift and steer accordingly. The only trusted speed source is
// localization (successive fixes ⇒ dx/dt). If localization is stale, the
// reported speed is 0; we do NOT extrapolate motion from inertial data.
//
// Yaw is gyro-integrated and corrected by the STM32-fused orientation:
//   prediction:   yaw(k+1) = yaw(k) + (gyro_z - yaw_bias) * dt
//   measurement:  yaw_meas = tf2::getYaw(STM32_quaternion)
//   gates:        non-finite quat, ‖q‖∉[0.95,1.05], cov[8]>0.5 ⇒ reject
//
// Localization corrects but never drives:
//   - successive fixes yield v_loc_filtered (low-pass 0.2/0.8 per spec)
//   - v_blended = v_loc_filtered when loc fresh, 0 otherwise
//   - EKF position is dead-reckoned by v_blended along yaw between fixes,
//     and corrected by a (x, y) EKF update when a fix arrives.
//
// IMU orientation is a MEASUREMENT, never a state replacement (Phase 2.5):
//   The STM32 already runs onboard sensor fusion (gyro + accel + mag) and
//   exposes a fused orientation quaternion on /microros/imu_raw together with
//   covariances. We therefore do NOT re-estimate orientation here; instead we
//   ingest the STM32 quaternion as a yaw measurement and apply a standard
//   Kalman update against the predicted yaw. The state transition still uses
//   the bias-corrected gyro for prediction:
//       yaw(k+1) = yaw(k) + (gyro_z - yaw_bias) * dt
//   The measurement R is scaled by an adaptive imu_confidence derived from
//   the live covariance triplet, so noisy IMU samples auto-attenuate without
//   needing a hard reject on every borderline frame.
//
// Confidence is a smoothly-decaying weighted combination of
//   0.4*imu_health + 0.3*loc_health + 0.3*covariance_health
// — never snaps to zero.
class EkfLocalizationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EkfLocalizationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void correction_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void localization_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timer_step();
  double normalize_yaw(double yaw);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drift_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr loc_sub_;
  rclcpp_lifecycle::LifecyclePublisher<
      bfmc_msgs::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Phase 2.5: IMU adaptive diagnostics. /fusion/imu_confidence is always-on
  // (downstream supervisor can use it). The three /fusion/debug/* mirror
  // are gated by enable_debug_topics_ — symmetric with drift_estimator.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_yaw_measurement_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_yaw_bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_imu_confidence_pub_;

  // IMU runtime
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_imu_stamp_seen_;     // wall-clock of last imu_callback (for imu_health)
  bool imu_received_{false};
  double latest_correction_{0.0};
  sensor_msgs::msg::Imu latest_imu_;

  // Speed estimation — localization-only (Phase 2.6).
  //   v_loc_filtered : low-passed dx/dt from successive localization fixes.
  //   v_blended      : published speed = v_loc_filtered when loc fresh, else 0.
  // accel-derived speed (a_filtered_, imu_speed_) has been removed; see the
  // class-header doc-block for the rationale.
  double v_loc_filtered_{0.0};
  double v_blended_{0.0};

  // Localization runtime
  bool loc_received_{false};
  bool loc_have_prev_{false};
  bool loc_pending_{false};
  double prev_loc_x_{0.0};
  double prev_loc_y_{0.0};
  double pending_loc_x_{0.0};
  double pending_loc_y_{0.0};
  rclcpp::Time prev_loc_stamp_;
  rclcpp::Time last_loc_stamp_;

  // Output buffer
  bfmc_msgs::msg::VehicleState state_msg_;

  // EKF matrices — state x = [x, y, yaw, yaw_bias]
  Eigen::Matrix<double, 4, 1> x_;
  Eigen::Matrix<double, 4, 4> P_;
  Eigen::Matrix<double, 4, 4> Q_;
  Eigen::Matrix<double, 4, 4> F_;
  Eigen::Matrix<double, 4, 4> I_;
  Eigen::Matrix<double, 1, 1> R_yaw_;
  Eigen::Matrix<double, 2, 2> R_xy_;
  Eigen::Matrix<double, 1, 4> H_yaw_;
  Eigen::Matrix<double, 2, 4> H_xy_;

  // Phase 2.5: latest computed adaptive IMU confidence, in [0, 1]. 1.0 ⇒ all
  // three covariance triplets near zero; ~0 ⇒ very noisy IMU. Used to scale
  // R_yaw per measurement and re-published for downstream consumers.
  double imu_confidence_{1.0};

  // Phase 2.5: enable_debug_topics gate for /fusion/debug/{yaw_measurement,
  // yaw_bias, imu_confidence}. Defaults to false; the always-on
  // /fusion/imu_confidence stream is separate and lives regardless.
  bool enable_debug_topics_{false};

  // ====== Constants (tuned for BFMC) ======
  static constexpr double MAX_DT         = 0.1;
  static constexpr double IMU_TIMEOUT    = 0.5;
  static constexpr double MAX_VEL        = 2.0;
  static constexpr double YAW_GATE       = 0.5;
  static constexpr double LOC_TIMEOUT    = 3.0;   // dead-reckon grace (s)
  static constexpr double LOC_STD        = 0.15;  // 15 cm per spec
  static constexpr double V_LOC_ALPHA    = 0.2;   // v_loc = .2*v + .8*v_prev
  // Confidence
  static constexpr double IMU_HEALTH_TAU = 0.5;    // health=1 at age 0, =0 at age TAU
  static constexpr double COV_POS_MAX    = 1.0;    // sum P(0,0)+P(1,1) considered "bad"
  static constexpr double W_IMU_HEALTH   = 0.4;
  static constexpr double W_LOC_HEALTH   = 0.3;
  static constexpr double W_COV_HEALTH   = 0.3;

  // Phase 2.5: orientation-measurement adaptive R + sanity gates.
  //   R_YAW_BASE     baseline measurement noise variance (rad^2) when IMU is
  //                  reporting healthy covariances; previously the value of
  //                  R_yaw_(0,0). Scaled per-tick by 1/max(imu_conf, MIN).
  //   ORIENT_COV_MAX hard reject if orientation_covariance[8] > this (very
  //                  uncertain yaw — STM32 itself thinks the estimate is bad).
  //   QUAT_NORM_*    accept window for ‖q‖. Anything outside [0.95, 1.05]
  //                  indicates serialization corruption or unfused stream.
  //   IMU_CONF_MIN   floor on the divisor to bound R_yaw_adaptive even when
  //                  imu_confidence collapses near zero.
  static constexpr double R_YAW_BASE     = 0.05;
  static constexpr double ORIENT_COV_MAX = 0.5;
  static constexpr double QUAT_NORM_MIN  = 0.95;
  static constexpr double QUAT_NORM_MAX  = 1.05;
  static constexpr double IMU_CONF_MIN   = 0.1;

  // Phase 2.6: weighted IMU confidence — orientation dominates because yaw
  // is the only IMU-derived state we trust; accel barely matters now that we
  // no longer integrate it for speed.
  //   imu_confidence = exp(-(W_CONF_ORIENT*σ_orient
  //                        + W_CONF_GYRO  *σ_gyro
  //                        + W_CONF_ACCEL *σ_accel))
  static constexpr double W_CONF_ORIENT = 0.6;
  static constexpr double W_CONF_GYRO   = 0.3;
  static constexpr double W_CONF_ACCEL  = 0.1;

  // Phase 2.6: CSV logging plumbing. Disabled by default; enable through the
  // ROS parameter `enable_csv_logging`. Writes to `csv_log_dir`
  // (default /var/log/bfmc/fusion). Buffered, append-mode, flushed every
  // CSV_FLUSH_EVERY ticks.
  bool enable_csv_logging_{false};
  std::string csv_log_dir_{"/var/log/bfmc/fusion"};
  std::ofstream csv_imu_confidence_;
  std::ofstream csv_yaw_measurement_;
  std::ofstream csv_yaw_bias_;
  std::ofstream csv_vehicle_state_;
  std::uint64_t csv_tick_counter_{0};
  static constexpr std::uint64_t CSV_FLUSH_EVERY = 100;   // 100 ticks @100Hz = 1 s

  bool open_csv_logs_();
  void close_csv_logs_();
  // logger always sees the latest measurement context; pass nan when invalid.
  void write_csv_row_(double t,
                      double yaw_measurement,
                      double yaw_bias,
                      double imu_conf,
                      double veh_yaw,
                      double veh_speed,
                      bool   yaw_meas_valid);
};

}  // namespace bfmc_fusion

#endif  // EKF_LOCALIZATION__EKF_LOCALIZATION_NODE_HPP_
