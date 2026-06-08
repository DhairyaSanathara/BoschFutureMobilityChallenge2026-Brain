#include "drift_estimator/drift_estimator_node.hpp"
#include <cmath>
#include <algorithm>

namespace bfmc_fusion
{

DriftEstimatorNode::DriftEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("drift_estimator_node", options),
  state_received_(false), lane_received_(false), e_filtered_(0.0), previous_correction_(0.0) {}

CallbackReturn DriftEstimatorNode::on_configure(const rclcpp_lifecycle::State &)
{
  auto state_qos = rclcpp::QoS(5).reliable();
  auto lane_qos  = rclcpp::QoS(5).best_effort();
  auto pub_qos   = rclcpp::QoS(10).reliable();
  auto dbg_qos   = rclcpp::QoS(5).best_effort();

  state_sub_ = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", state_qos, std::bind(&DriftEstimatorNode::vehicle_state_callback, this, std::placeholders::_1));

  lane_sub_ = create_subscription<bfmc_msgs::msg::LaneGeometry>(
    "/perception/lane_geometry", lane_qos, std::bind(&DriftEstimatorNode::lane_geometry_callback, this, std::placeholders::_1));

  correction_pub_ = create_publisher<std_msgs::msg::Float64>("/fusion/yaw_bias_correction", pub_qos);

  // --- Parameters -----------------------------------------------------------
  // All defaults come from drift_estimator_node.hpp. Parameters allow race-day
  // tuning (lookahead row, sign, confidence threshold, etc.) without rebuilds.
  lookahead_row_px_     = declare_parameter<double>("lookahead_row_px",     lookahead_row_px_);
  lookahead_m_          = declare_parameter<double>("lookahead_m",          lookahead_m_);
  lane_conf_thresh_     = declare_parameter<double>("lane_conf_thresh",     lane_conf_thresh_);
  max_curvature_coeff_  = declare_parameter<double>("max_curvature_coeff",  max_curvature_coeff_);
  error_sign_           = declare_parameter<double>("error_sign",           error_sign_);
  enable_debug_topics_  = declare_parameter<bool>("enable_debug_topics",    enable_debug_topics_);
  stale_state_max_s_    = declare_parameter<double>("stale_state_max_s",    stale_state_max_s_);
  stale_lane_max_s_     = declare_parameter<double>("stale_lane_max_s",     stale_lane_max_s_);

  // --- Lazy debug publishers ------------------------------------------------
  if (enable_debug_topics_) {
    debug_weighted_error_pub_ =
      create_publisher<std_msgs::msg::Float64>("/fusion/debug/weighted_error",   dbg_qos);
    debug_curvature_weight_pub_ =
      create_publisher<std_msgs::msg::Float64>("/fusion/debug/curvature_weight", dbg_qos);
    debug_lane_heading_pub_ =
      create_publisher<std_msgs::msg::Float64>("/fusion/debug/lane_heading",     dbg_qos);
    debug_node_latency_pub_ =
      create_publisher<std_msgs::msg::Float64>("/fusion/debug/node_latency_ms",  dbg_qos);
    debug_message_age_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/fusion/debug/message_age_ms", dbg_qos);
    debug_correction_active_pub_ =
      create_publisher<std_msgs::msg::Bool>("/fusion/debug/correction_active",   dbg_qos);
    RCLCPP_INFO(get_logger(),
      "drift_estimator: debug topics ENABLED "
      "(/fusion/debug/{weighted_error,curvature_weight,lane_heading,"
      "node_latency_ms,message_age_ms,correction_active})");
  } else {
    RCLCPP_INFO(get_logger(),
      "drift_estimator: debug topics disabled (set enable_debug_topics:=true to enable)");
  }

  RCLCPP_INFO(get_logger(),
    "drift_estimator config: lookahead_row_px=%.1f (lookahead_m=%.2f), "
    "lane_conf_thresh=%.2f, max_curvature_coeff=%.4f, error_sign=%.0f, "
    "stale_state_max=%.0f ms, stale_lane_max=%.0f ms",
    lookahead_row_px_, lookahead_m_, lane_conf_thresh_,
    max_curvature_coeff_, error_sign_,
    stale_state_max_s_ * 1000.0, stale_lane_max_s_ * 1000.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DriftEstimatorNode::on_activate(const rclcpp_lifecycle::State &)
{
  correction_pub_->on_activate();
  state_received_ = false; lane_received_ = false;
  e_filtered_ = 0.0; previous_correction_ = 0.0;

  // Seed all age clocks to "now" so the first 100 ms after activation does not
  // immediately log a stale-input warning before the first callback arrives.
  const auto t = now();
  last_state_recv_ = t;
  last_lane_recv_  = t;
  last_stale_warn_ = t;

  // 20 Hz loop
  loop_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&DriftEstimatorNode::timer_step,this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn DriftEstimatorNode::on_deactivate(const rclcpp_lifecycle::State &) { loop_timer_.reset(); correction_pub_->on_deactivate(); return CallbackReturn::SUCCESS; }
CallbackReturn DriftEstimatorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  state_sub_.reset();
  lane_sub_.reset();
  correction_pub_.reset();
  debug_weighted_error_pub_.reset();
  debug_curvature_weight_pub_.reset();
  debug_lane_heading_pub_.reset();
  debug_node_latency_pub_.reset();
  debug_message_age_pub_.reset();
  debug_correction_active_pub_.reset();
  return CallbackReturn::SUCCESS;
}
CallbackReturn DriftEstimatorNode::on_shutdown(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

void DriftEstimatorNode::vehicle_state_callback(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  latest_state_ = *msg;
  state_received_ = true;
  last_state_recv_ = now();   // wall-clock receive time, used for stale-input gate
}
void DriftEstimatorNode::lane_geometry_callback(const bfmc_msgs::msg::LaneGeometry::SharedPtr msg)
{
  latest_lane_ = *msg;
  lane_received_ = true;
  last_lane_recv_ = now();    // wall-clock receive time, used for stale-input gate
}

double DriftEstimatorNode::normalize_yaw(double yaw)
{
  while (yaw > M_PI) yaw -= 2.0*M_PI;
  while (yaw < -M_PI) yaw += 2.0*M_PI;
  return yaw;
}

void DriftEstimatorNode::timer_step()
{
  // Latency timing wraps the entire tick body. Published to
  // /fusion/debug/node_latency_ms when debug enabled.
  const auto tick_start = now();

  if(!state_received_ || !lane_received_) return;

  // ----- Message-age protection (TASK 3) ------------------------------------
  // bfmc_msgs/VehicleState and bfmc_msgs/LaneGeometry carry no header.stamp,
  // so we measure age from the wall-clock receive time recorded in each
  // callback. This catches the failure mode "upstream stopped publishing" —
  // which is what 'stale data' means at this layer of the graph.
  const double state_age_s = (tick_start - last_state_recv_).seconds();
  const double lane_age_s  = (tick_start - last_lane_recv_ ).seconds();

  const bool state_fresh = state_age_s <= stale_state_max_s_;
  const bool lane_fresh  = lane_age_s  <= stale_lane_max_s_;
  const bool inputs_fresh = state_fresh && lane_fresh;

  if (!inputs_fresh) {
    // Throttled to 1 Hz (STALE_WARN_PERIOD_S) to keep the log readable.
    if ((tick_start - last_stale_warn_).seconds() >= STALE_WARN_PERIOD_S) {
      RCLCPP_WARN(get_logger(),
        "stale input dropped: state_age=%.1f ms (max %.0f), lane_age=%.1f ms (max %.0f)",
        state_age_s * 1000.0, stale_state_max_s_ * 1000.0,
        lane_age_s  * 1000.0, stale_lane_max_s_  * 1000.0);
      last_stale_warn_ = tick_start;
    }
  }

  // ----- Runtime sanity guards (TASK D) -------------------------------------
  // Reject malformed lane data before doing any arithmetic. On rejection we
  // fall through to the decay branch via `safe = false`.
  const bool lane_data_ok =
    latest_lane_.center_poly.size() >= 3
    && std::isfinite(latest_lane_.center_poly[0])
    && std::isfinite(latest_lane_.center_poly[1])
    && std::isfinite(latest_lane_.center_poly[2])
    && std::abs(latest_lane_.center_poly[2]) <= max_curvature_coeff_;

  // ----- Safety gate --------------------------------------------------------
  // Now AND'd with input freshness so a stalled upstream forces decay.
  const bool safe =
    inputs_fresh
    && lane_data_ok
    && latest_lane_.confidence > lane_conf_thresh_
    && latest_state_.confidence > 0.0
    && std::abs(latest_state_.speed) > SPEED_THRESH;

  double correction = 0.0;

  // Debug placeholders — published every tick regardless of branch so that
  // ros2 topic echo / ros2 topic hz keep streaming.
  double dbg_weighted_error   = 0.0;
  double dbg_curvature_weight = 0.0;
  double dbg_lane_heading     = 0.0;

  if(safe)
  {
    // -----------------------------------------------------------------------
    // UNITS CHAIN (post-fortification)
    //
    // Lane fit:  x_image = c0 + c1*y_image + c2*y_image^2
    //   x_image: column index in [0, 640) px
    //   y_image: row    index in [144, 360) px   (cropped top removed)
    //   c0, c1, c2: dimensions chosen so the above equation balances in px.
    //
    // Heading at lookahead row y_eval (pixels):
    //   slope_image = dx_image/dy_image |_{y=y_eval}  =  c1 + 2*c2*y_eval
    //
    // The IMAGE slope is taken as an APPROXIMATION of the vehicle-frame
    // dY_vehicle/dX_vehicle (no IPM scale applied). Sign analysis:
    //   lane bends RIGHT in world  ⇒  slope_image < 0  ⇒  psi_lane < 0
    //   yaw_bias must INCREASE (so corrected yaw_rate = gyro - bias drops,
    //   vehicle turns right)        ⇒  correction must be POSITIVE
    //   ⇒ error = -psi_lane  (parameterized as error = error_sign * psi_lane,
    //                         default error_sign = -1.0).
    //
    // The previous "error = psi_lane - latest_state_.yaw" was wrong: psi_lane
    // is vehicle-relative and state_.yaw is world-frame, so the subtraction
    // closed a positive-feedback loop on the bias estimator. Removed.
    // -----------------------------------------------------------------------
    const double c1 = latest_lane_.center_poly[1];
    const double c2 = latest_lane_.center_poly[2];

    const double slope_image = c1 + 2.0 * c2 * lookahead_row_px_;
    const double psi_lane    = std::atan(slope_image);   // radians, vehicle-frame approx
    const double error       = error_sign_ * psi_lane;   // sign convention above

    // Phase 2 soft weight (already in place): weight = exp(-K_CURVATURE * |c2|), k=5.
    const double weight          = std::exp(-K_CURVATURE * std::abs(c2));
    const double weighted_error  = weight * error;

    // Preserve EMA on the (weighted) error and BETA gain.
    e_filtered_ = ALPHA * weighted_error + (1.0 - ALPHA) * e_filtered_;
    correction  = BETA * e_filtered_;

    dbg_weighted_error   = weighted_error;
    dbg_curvature_weight = weight;
    dbg_lane_heading     = psi_lane;
  }
  else
  {
    correction = DECAY_FACTOR * previous_correction_;
    e_filtered_ *= DECAY_FACTOR;
  }

  // Hard Clamp
  correction = std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);
  previous_correction_ = correction;

  correction_msg_.data = correction;
  correction_pub_->publish(correction_msg_);

  // Emit debug stream when enabled.
  if (enable_debug_topics_) {
    std_msgs::msg::Float64 m;
    m.data = dbg_weighted_error;
    debug_weighted_error_pub_->publish(m);
    m.data = dbg_curvature_weight;
    debug_curvature_weight_pub_->publish(m);
    m.data = dbg_lane_heading;
    debug_lane_heading_pub_->publish(m);

    // Runtime diagnostics (TASK 4).
    const auto tick_end = now();
    const double latency_ms = (tick_end - tick_start).seconds() * 1000.0;

    std_msgs::msg::Float64 lat;
    lat.data = latency_ms;
    debug_node_latency_pub_->publish(lat);

    std_msgs::msg::Float32MultiArray ages;
    ages.data = {
      static_cast<float>(state_age_s * 1000.0),
      static_cast<float>(lane_age_s  * 1000.0)
    };
    debug_message_age_pub_->publish(ages);

    std_msgs::msg::Bool active;
    active.data = safe;
    debug_correction_active_pub_->publish(active);
  }
}
} // namespace bfmc_fusion
