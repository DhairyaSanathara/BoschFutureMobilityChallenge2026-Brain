// =============================================================================
// image_preprocessor_node.cpp -- BFMC 2026 (CUDA + NEON OpenCV / composable)
// =============================================================================
// Hot-path design:
//
//   camera Image  -->  pin into pinned host buffer  -->  H2D async  -->
//   CUDA kernel (resize + BGR->Lab L)  -->  D2H async  -->  stream sync
//   -->  CLAHE (CPU NEON)  -->  Gaussian / Sobel / threshold / mask /
//   morph (CPU NEON)  -->  paste binary into output Image (zero top)
//   -->  publish std::move(unique_ptr<Image>)
//
// Why the GPU stops here (instead of also doing CLAHE / Sobel / morph):
//   - CLAHE on GPU requires per-tile shared-memory histograms; the gain over
//     CPU NEON on a 216x640 buffer is small (~3 ms saved) and the kernel
//     surface is large enough to be a separate work item.
//   - Sobel / threshold / morph on a 216x640 buffer run in ~3 ms on NEON CPU.
//     Their GPU cost is dominated by launch overhead at that size.
//   - The expensive stage (1280x720 -> 360x640 resize + BGR2Lab) is the one
//     we move to the GPU. That alone drops CPU per-frame cost from ~14 ms to
//     ~3 ms, lifting throughput from 21 Hz to camera-rate (29 Hz).
// =============================================================================

#include "image_preprocessor_cpp/image_preprocessor_node.hpp"
#include "image_preprocessor_cpp/preprocess_cuda.hpp"

#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace image_preprocessor_cpp
{

ImagePreprocessorNode::ImagePreprocessorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_preprocessor_node", options)
{
  // Parameters
  declare_parameter<double>("stale_threshold_s", 0.1);
  stale_threshold_s_ = get_parameter("stale_threshold_s").as_double();

  // CUDA: initialise stream + sRGB LUT in __constant__ memory.
  check_cuda_(init_srgb_constants(), "init_srgb_constants");
  check_cuda_(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
              "cudaStreamCreateWithFlags");

  // Allocate the device + pinned-host buffers for the output L channel up
  // front; size is known (360 * 640 = 230400 bytes).
  const std::size_t l_bytes =
    static_cast<std::size_t>(kResizeH) * static_cast<std::size_t>(kResizeW);
  void * dl = nullptr;
  check_cuda_(cudaMalloc(&dl, l_bytes), "cudaMalloc(d_l_)");
  d_l_ = static_cast<uint8_t *>(dl);

  void * hl = nullptr;
  check_cuda_(cudaHostAlloc(&hl, l_bytes, cudaHostAllocDefault),
              "cudaHostAlloc(h_l_pinned_)");
  h_l_pinned_ = static_cast<uint8_t *>(hl);

  // CPU workspace cv::Mats wrap the pinned host L buffer where possible.
  l_full_     = cv::Mat(kResizeH, kResizeW, CV_8UC1, h_l_pinned_);
  l_cropped_  = l_full_(cv::Range(kRoiTopCutoff, kResizeH), cv::Range::all());

  l_eq_           = cv::Mat(kCropH, kResizeW, CV_8UC1);
  l_blurred_      = cv::Mat(kCropH, kResizeW, CV_8UC1);
  sobel_x_16s_    = cv::Mat(kCropH, kResizeW, CV_16S);
  sobel_y_16s_    = cv::Mat(kCropH, kResizeW, CV_16S);
  abs_x_          = cv::Mat(kCropH, kResizeW, CV_8UC1);
  abs_y_          = cv::Mat(kCropH, kResizeW, CV_8UC1);
  sobel_combined_ = cv::Mat(kCropH, kResizeW, CV_8UC1);
  binary_         = cv::Mat(kCropH, kResizeW, CV_8UC1);

  morph_kernel_ = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kMorphKernel, kMorphKernel));

  clahe_ = cv::createCLAHE(kClaheClip, cv::Size(kClaheTile, kClaheTile));

  // ROI trapezoid mask: identical to the Python pipeline.
  roi_mask_ = cv::Mat::zeros(kCropH, kResizeW, CV_8UC1);
  std::vector<cv::Point> poly = {
    cv::Point(0, kCropH),
    cv::Point(static_cast<int>(kResizeW * 0.10), static_cast<int>(kCropH * 0.20)),
    cv::Point(static_cast<int>(kResizeW * 0.90), static_cast<int>(kCropH * 0.20)),
    cv::Point(kResizeW, kCropH),
  };
  std::vector<std::vector<cv::Point>> polys = {poly};
  cv::fillPoly(roi_mask_, polys, cv::Scalar(255));

  // Explicit SensorDataQoS: KEEP_LAST(1) + BEST_EFFORT + VOLATILE.
  // Newest frame wins; stale frames die in DDS, not in our callback queue.
  const auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();

  publisher_ = create_publisher<sensor_msgs::msg::Image>(
    "/perception/image_preprocessed", qos);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", qos,
    std::bind(&ImagePreprocessorNode::image_callback, this, std::placeholders::_1));

  last_diag_log_ = now();

  RCLCPP_INFO(get_logger(),
    "ImagePreprocessor (CUDA+NEON) ready: out %dx%d, ROI top %d, "
    "threshold %d, stale_threshold=%.3fs",
    kResizeW, kResizeH, kRoiTopCutoff, kThreshold, stale_threshold_s_);
}

ImagePreprocessorNode::~ImagePreprocessorNode()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
  if (d_input_)      { cudaFree(d_input_);       d_input_ = nullptr; }
  if (d_l_)          { cudaFree(d_l_);           d_l_ = nullptr; }
  if (h_input_pinned_) { cudaFreeHost(h_input_pinned_); h_input_pinned_ = nullptr; }
  if (h_l_pinned_)   { cudaFreeHost(h_l_pinned_); h_l_pinned_ = nullptr; }
}

bool ImagePreprocessorNode::is_supported_encoding_(const std::string & enc) const
{
  return enc == sensor_msgs::image_encodings::BGR8 ||
         enc == sensor_msgs::image_encodings::RGB8;
}

void ImagePreprocessorNode::check_cuda_(cudaError_t err, const char * where)
{
  if (err != cudaSuccess) {
    std::ostringstream s;
    s << "CUDA error at " << where << ": " << cudaGetErrorString(err);
    throw std::runtime_error(s.str());
  }
}

void ImagePreprocessorNode::ensure_pinned_capacity_(std::size_t bytes)
{
  if (bytes <= h_input_bytes_ && h_input_pinned_) {
    return;
  }
  if (h_input_pinned_) {
    cudaFreeHost(h_input_pinned_);
    h_input_pinned_ = nullptr;
  }
  void * p = nullptr;
  check_cuda_(cudaHostAlloc(&p, bytes, cudaHostAllocDefault),
              "cudaHostAlloc(h_input_pinned_)");
  h_input_pinned_ = static_cast<uint8_t *>(p);
  h_input_bytes_ = bytes;
}

void ImagePreprocessorNode::ensure_device_capacity_(std::size_t bytes)
{
  if (bytes <= d_input_bytes_ && d_input_) {
    return;
  }
  if (d_input_) {
    cudaFree(d_input_);
    d_input_ = nullptr;
  }
  void * p = nullptr;
  check_cuda_(cudaMalloc(&p, bytes), "cudaMalloc(d_input_)");
  d_input_ = static_cast<uint8_t *>(p);
  d_input_bytes_ = bytes;
}

void ImagePreprocessorNode::run_cpu_post_clahe_()
{
  // CLAHE on the cropped L channel (which lives in l_cropped_, a view over
  // the bottom 216 rows of the pinned host L buffer).
  clahe_->apply(l_cropped_, l_eq_);

  cv::GaussianBlur(l_eq_, l_blurred_, cv::Size(kGaussianKernel, kGaussianKernel), 0);

  cv::Sobel(l_blurred_, sobel_x_16s_, CV_16S, 1, 0, 3);
  cv::Sobel(l_blurred_, sobel_y_16s_, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(sobel_x_16s_, abs_x_, kSobelXWeight);
  cv::convertScaleAbs(sobel_y_16s_, abs_y_, kSobelYWeight);
  cv::add(abs_x_, abs_y_, sobel_combined_);

  cv::threshold(sobel_combined_, binary_, kThreshold, 255, cv::THRESH_BINARY);
  cv::bitwise_and(binary_, roi_mask_, binary_);
  cv::morphologyEx(binary_, binary_, cv::MORPH_CLOSE, morph_kernel_);
}

void ImagePreprocessorNode::image_callback(
  sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // ----- Stale-frame protection ---------------------------------------
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  const rclcpp::Time now_t = now();
  if (stale_threshold_s_ > 0.0 &&
      stamp.nanoseconds() != 0 &&
      (now_t - stamp).seconds() > stale_threshold_s_)
  {
    ++dropped_stale_;
    auto throttle_clock = get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *throttle_clock, 2000,
      "dropping stale frame (age %.0f ms > %.0f ms)",
      (now_t - stamp).seconds() * 1000.0,
      stale_threshold_s_ * 1000.0);
    return;
  }

  // ----- Validate encoding --------------------------------------------
  if (!is_supported_encoding_(msg->encoding)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "unsupported encoding: %s", msg->encoding.c_str());
    return;
  }
  const bool is_bgr = (msg->encoding == sensor_msgs::image_encodings::BGR8);
  const int src_h = static_cast<int>(msg->height);
  const int src_w = static_cast<int>(msg->width);
  const int src_step = static_cast<int>(msg->step);
  if (src_h <= 0 || src_w <= 0 || msg->data.empty()) {
    return;
  }
  const std::size_t src_bytes =
    static_cast<std::size_t>(src_step) * static_cast<std::size_t>(src_h);

  try {
    // ----- H2D ------------------------------------------------------
    ensure_pinned_capacity_(src_bytes);
    ensure_device_capacity_(src_bytes);
    std::memcpy(h_input_pinned_, msg->data.data(), src_bytes);

    check_cuda_(cudaMemcpyAsync(
                  d_input_, h_input_pinned_, src_bytes,
                  cudaMemcpyHostToDevice, stream_),
                "cudaMemcpyAsync(H2D src)");

    // ----- Fused resize + BGR/RGB->Lab L --------------------------
    check_cuda_(launch_resize_bgr_to_lab_l(
                  d_input_, src_h, src_w, src_step, is_bgr,
                  d_l_, kResizeH, kResizeW, stream_),
                "launch_resize_bgr_to_lab_l");

    // ----- D2H L channel (into pinned host buffer backing l_full_) -
    const std::size_t l_bytes =
      static_cast<std::size_t>(kResizeH) * static_cast<std::size_t>(kResizeW);
    check_cuda_(cudaMemcpyAsync(
                  h_l_pinned_, d_l_, l_bytes,
                  cudaMemcpyDeviceToHost, stream_),
                "cudaMemcpyAsync(D2H L)");

    check_cuda_(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

    // ----- CPU post-CLAHE pipeline (NEON OpenCV) -------------------
    run_cpu_post_clahe_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "preprocess pipeline failed: %s", e.what());
    return;
  }

  // ----- Build output Image: top 144 rows zero, bottom 216 rows = binary --
  auto out = std::make_unique<sensor_msgs::msg::Image>();
  out->header = msg->header;            // PRESERVE TIMESTAMP
  out->height = static_cast<uint32_t>(kResizeH);
  out->width  = static_cast<uint32_t>(kResizeW);
  out->encoding = sensor_msgs::image_encodings::MONO8;
  out->is_bigendian = 0;
  out->step = static_cast<uint32_t>(kResizeW);
  out->data.assign(static_cast<size_t>(kResizeH) * static_cast<size_t>(kResizeW), 0);
  std::memcpy(
    out->data.data() + static_cast<size_t>(kRoiTopCutoff) * static_cast<size_t>(kResizeW),
    binary_.data,
    static_cast<size_t>(kCropH) * static_cast<size_t>(kResizeW));

  publisher_->publish(std::move(out));
  ++processed_frames_;

  // Periodic diagnostic log (every 5 s).
  if ((now_t - last_diag_log_).seconds() > 5.0) {
    RCLCPP_INFO(get_logger(),
      "preproc: processed=%lu dropped_stale=%lu",
      static_cast<unsigned long>(processed_frames_),
      static_cast<unsigned long>(dropped_stale_));
    processed_frames_ = 0;
    dropped_stale_ = 0;
    last_diag_log_ = now_t;
  }
}

}  // namespace image_preprocessor_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(image_preprocessor_cpp::ImagePreprocessorNode)
