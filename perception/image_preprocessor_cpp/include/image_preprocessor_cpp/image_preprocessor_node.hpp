#ifndef IMAGE_PREPROCESSOR_CPP__IMAGE_PREPROCESSOR_NODE_HPP_
#define IMAGE_PREPROCESSOR_CPP__IMAGE_PREPROCESSOR_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cuda_runtime.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace image_preprocessor_cpp
{

// BFMC 2026 image preprocessor (CUDA + NEON OpenCV / composable).
//
// GPU path (one fused kernel, async):
//   resize(any -> 640x360) + (BGR|RGB) -> Lab L*  (uint8 [0..255])
//
// CPU path (NEON OpenCV, runs on the small 360x640 mono buffer):
//   crop(144..360) -> CLAHE -> Gaussian(5x5) -> Sobel x/y(16S)
//   -> convertScaleAbs * 0.7 / 0.3 -> add -> threshold(140)
//   -> AND(roi_mask) -> morphologyEx(CLOSE 3x3) -> paste back top-zeroed
//
// Output: mono8 360x640 with input header propagated verbatim.
//
// Stale-frame protection: configurable parameter `stale_threshold_s`
// (default 0.1 s). Frames whose header.stamp is older than that are dropped
// before any CUDA / CPU work, with throttled logging.
class ImagePreprocessorNode : public rclcpp::Node
{
public:
  explicit ImagePreprocessorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImagePreprocessorNode() override;

private:
  static constexpr int kResizeW = 640;
  static constexpr int kResizeH = 360;
  static constexpr int kRoiTopCutoff = 144;            // 360 * 0.40
  static constexpr int kCropH = kResizeH - kRoiTopCutoff;   // 216
  static constexpr int kThreshold = 140;
  static constexpr int kMorphKernel = 3;
  static constexpr double kClaheClip = 2.0;
  static constexpr int kClaheTile = 8;
  static constexpr double kSobelXWeight = 0.7;
  static constexpr double kSobelYWeight = 0.3;
  static constexpr int kGaussianKernel = 5;

  void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  bool is_supported_encoding_(const std::string & enc) const;
  void ensure_pinned_capacity_(std::size_t bytes);
  void ensure_device_capacity_(std::size_t bytes);
  void check_cuda_(cudaError_t err, const char * where);
  void run_cpu_post_clahe_();   // operates on l_eq_ -> binary_

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  // -------- CUDA resources -------------------------------------------------
  cudaStream_t stream_{nullptr};
  uint8_t * h_input_pinned_{nullptr};   // page-locked staging for camera frame
  std::size_t h_input_bytes_{0};
  uint8_t * d_input_{nullptr};
  std::size_t d_input_bytes_{0};
  uint8_t * d_l_{nullptr};              // resized L channel (360 * 640 = 230400)
  uint8_t * h_l_pinned_{nullptr};       // pinned host buffer for D2H of L

  // -------- CPU workspace --------------------------------------------------
  // l_full_ is a cv::Mat view over h_l_pinned_ (no allocation; just a header).
  cv::Mat l_full_;            // 360x640 CV_8UC1 (top region unused after crop)
  cv::Mat l_cropped_;         // 216x640 CV_8UC1 view of bottom of l_full_
  cv::Mat l_eq_;              // 216x640 CV_8UC1 (CLAHE out)
  cv::Mat l_blurred_;         // 216x640 CV_8UC1
  cv::Mat sobel_x_16s_;       // 216x640 CV_16S
  cv::Mat sobel_y_16s_;       // 216x640 CV_16S
  cv::Mat abs_x_;             // 216x640 CV_8UC1
  cv::Mat abs_y_;             // 216x640 CV_8UC1
  cv::Mat sobel_combined_;    // 216x640 CV_8UC1
  cv::Mat binary_;            // 216x640 CV_8UC1
  cv::Mat roi_mask_;          // 216x640 CV_8UC1
  cv::Mat morph_kernel_;
  cv::Ptr<cv::CLAHE> clahe_;

  // -------- stale-frame protection ----------------------------------------
  double stale_threshold_s_{0.1};

  // -------- diagnostic counters (logged at most every 5 s) ----------------
  std::uint64_t dropped_stale_{0};
  std::uint64_t processed_frames_{0};
  rclcpp::Time last_diag_log_;
};

}  // namespace image_preprocessor_cpp

#endif  // IMAGE_PREPROCESSOR_CPP__IMAGE_PREPROCESSOR_NODE_HPP_
