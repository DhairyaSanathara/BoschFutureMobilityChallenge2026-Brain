#ifndef LANE_GEOMETRY_NODE_HPP_
#define LANE_GEOMETRY_NODE_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <bfmc_msgs/msg/lane_geometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace lane_geometry_cpp
{

class LaneGeometryNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LaneGeometryNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LaneGeometryNode() override;

protected:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Histogram-base + dual sliding-window extraction. Writes detected
  // lane-pixel coordinates into lane_points_ (cleared by caller).
  void extract_lane_points_sliding_window_(const cv::Mat & binary);

  // Cholesky normal-equations polyfit. Uses preallocated ATA_/ATb_ members.
  bool fit_poly_normal_equations_(
    const std::vector<cv::Point> & points,
    std::array<double, 3> & coeffs);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp_lifecycle::LifecyclePublisher<bfmc_msgs::msg::LaneGeometry>::SharedPtr
    geometry_pub_;

  // ------ sliding window config (compile-time defaults; tuned for 360x640) -
  int crop_top_y_{144};
  int n_windows_{9};
  int window_margin_{60};
  int min_pixels_{25};
  int base_search_rows_{60};
  int sample_stride_{2};

  // ------ EMA smoothing state ----------------------------------------------
  bool has_previous_curve_{false};
  std::array<double, 3> smoothed_coeffs_{{0.0, 0.0, 0.0}};
  double ema_alpha_{0.3};

  // ------ preallocated workspace -------------------------------------------
  std::vector<cv::Point> lane_points_;
  cv::Mat hist_row_;     // 1 x cols CV_32S
  cv::Mat ATA_;          // 3x3 CV_64F
  cv::Mat ATb_;          // 3x1 CV_64F
  cv::Mat coeffs_mat_;   // 3x1 CV_64F

  // ------ stale-frame protection ------------------------------------------
  double stale_threshold_s_{0.1};

  // ------ confidence state -------------------------------------------------
  // Filled by extract_lane_points_sliding_window_(): pixel counts per side,
  // used by compute_confidence_() to score edge_consistency.
  int last_left_pixels_{0};
  int last_right_pixels_{0};

  // Smoothed confidence — output field. EMA across frames so a single bad
  // frame cannot flap the value to zero.
  double confidence_{0.0};

  // Per-frame confidence factors (tuning knobs, no params yet).
  static constexpr double N_POINTS_SAT          = 150.0;  // lane_count_weight ramp
  static constexpr double RES_TAU_PX            = 30.0;   // fit_quality e-fold
  static constexpr double TEMPORAL_NORM_C0_PX   = 50.0;   // lateral tolerance
  static constexpr double TEMPORAL_NORM_C1      = 0.5;    // slope tolerance
  static constexpr double TEMPORAL_NORM_C2      = 0.005;  // curvature tolerance
  static constexpr double CONFIDENCE_EMA_ALPHA  = 0.3;    // 0.3*new + 0.7*prev
  static constexpr double CONFIDENCE_FAIL_DECAY = 0.7;    // on fit failure

  // Compute confidence in [0,1] from the just-extracted points and the
  // freshly-fit coefficients (pre-EMA). Caller blends with confidence_.
  double compute_confidence_(const std::array<double, 3> & current_coeffs) const;

  // ------ diagnostic counters (throttled log) -----------------------------
  std::uint64_t dropped_stale_{0};
  std::uint64_t processed_frames_{0};
  rclcpp::Time last_diag_log_;
};

}  // namespace lane_geometry_cpp

#endif  // LANE_GEOMETRY_NODE_HPP_
