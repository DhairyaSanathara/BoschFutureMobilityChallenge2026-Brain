#ifndef DRIFT_ESTIMATOR__DRIFT_ESTIMATOR_NODE_HPP_
#define DRIFT_ESTIMATOR__DRIFT_ESTIMATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"
#include "bfmc_msgs/msg/lane_geometry.hpp"

#include <memory>

namespace bfmc_fusion
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DriftEstimatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DriftEstimatorNode(
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  /* ---------- Lifecycle ---------- */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  /* ---------- Callbacks ---------- */
  void vehicle_state_callback(
      const bfmc_msgs::msg::VehicleState::SharedPtr msg);

  void lane_geometry_callback(
      const bfmc_msgs::msg::LaneGeometry::SharedPtr msg);

  void timer_step();
  double normalize_yaw(double yaw);

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<bfmc_msgs::msg::LaneGeometry>::SharedPtr lane_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr correction_pub_;

  // Optional debug publishers (lazy: only created if enable_debug_topics is true).
  // These are plain rclcpp::Publisher (not lifecycle) so they don't need to be
  // activated/deactivated alongside correction_pub_.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_weighted_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_curvature_weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_lane_heading_pub_;

  // Runtime diagnostics (also gated by enable_debug_topics_):
  //   /fusion/debug/node_latency_ms       Float64        timer_step wall time, ms
  //   /fusion/debug/message_age_ms        Float32[2]     [state_age_ms, lane_age_ms]
  //   /fusion/debug/correction_active     Bool           true ⇒ safe-branch taken this tick
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             debug_node_latency_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   debug_message_age_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                debug_correction_active_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;   // ⭐ FIX ADDED

  /* ---------- Cached Messages ---------- */
  bfmc_msgs::msg::VehicleState latest_state_;
  bfmc_msgs::msg::LaneGeometry latest_lane_;
  std_msgs::msg::Float64 correction_msg_;

  bool state_received_{false};
  bool lane_received_{false};

  /* ---------- Filter State ---------- */
  double e_filtered_{0.0};
  double previous_correction_{0.0};

  /* ---------- Runtime-tunable parameters ---------- */
  // Image row at which the lane-heading slope is evaluated. center_poly is in
  // IMAGE PIXEL UNITS (x_image = c0 + c1*y_image + c2*y_image^2 over y∈[144,360]).
  // Default 280 ≈ halfway through the visible lane mask, biased toward the
  // close-to-car region where the projective scale is most consistent.
  double lookahead_row_px_{280.0};

  // Semantic lookahead distance in METRES — informational only. Not used in
  // the heading formula (which works in image pixels). Held here so a later
  // IPM stage can derive lookahead_row_px_ from it. Default 0.5 m per spec.
  double lookahead_m_{0.5};

  // Confidence threshold on /perception/lane_geometry.confidence to enter
  // the active correction branch. Default lowered from 0.7 → 0.5 because the
  // Phase 1.5 confidence model produces EMA-smoothed values in ~[0.3, 0.6]
  // on routinely good frames.
  double lane_conf_thresh_{0.5};

  // Sanity cap on |c2|. Beyond this, the fit is treated as garbage and the
  // safe branch is skipped. Default 0.01 is ~3× the realistic worst case for
  // a BFMC track in image-pixel units.
  double max_curvature_coeff_{0.01};

  // Sign of the lane-heading error in the EKF's yaw_bias convention.
  // Derivation: image_u→right, image_v→down; vehicle_X→forward, vehicle_Y→left.
  //   A lane bending RIGHT in world ⇒ image slope < 0 ⇒ psi_lane < 0.
  //   To turn the car right (corrected yaw_rate = gyro - yaw_bias decreases),
  //   yaw_bias must INCREASE ⇒ correction must be POSITIVE.
  //   ⇒ error = -psi_lane (default error_sign = -1.0).
  // Parameterized so race-day calibration can flip polarity from CLI if a
  // logged drive shows divergence in the wrong direction.
  double error_sign_{-1.0};

  // Master switch for ALL /fusion/debug/* topics (heading, curvature_weight,
  // weighted_error, node_latency_ms, message_age_ms, correction_active).
  bool enable_debug_topics_{false};

  // Stale-input gates. bfmc_msgs/VehicleState and bfmc_msgs/LaneGeometry have
  // NO header.stamp field, so we cannot recover the producer's stamp. Instead
  // we record wall-clock receive time per source and reject if the most recent
  // sample is older than these thresholds — i.e. "the upstream stopped
  // publishing recently enough to trust the cached value." Defaults: 100 ms,
  // matching the 100 Hz EKF / 20 Hz lane producer cadence (≈10× publish period).
  double stale_state_max_s_{0.1};
  double stale_lane_max_s_ {0.1};

  // Last wall-clock receive timestamps and last-warning timestamp for the
  // 1 Hz throttled "stale input dropped" log.
  rclcpp::Time last_state_recv_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lane_recv_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stale_warn_{0, 0, RCL_ROS_TIME};

  /* ---------- Constants ---------- */
  static constexpr double ALPHA = 0.1;
  static constexpr double BETA = 0.3;
  static constexpr double MAX_CORRECTION = 0.005;
  static constexpr double SPEED_THRESH = 0.15;
  // Phase 2 soft weight: weight = exp(-K_CURVATURE * |c2|). k=5 per spec.
  // Replaces the prior hard if(|curvature| < CURVATURE_THRESH) gate.
  static constexpr double K_CURVATURE = 5.0;
  static constexpr double DECAY_FACTOR = 0.9;
  // Throttle period for the "stale input dropped" warning (1 Hz).
  static constexpr double STALE_WARN_PERIOD_S = 1.0;
};

} // namespace bfmc_fusion

#endif
