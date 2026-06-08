#ifndef OBJECT_DETECTION_CPP__OBJECT_DETECTION_NODE_HPP_
#define OBJECT_DETECTION_CPP__OBJECT_DETECTION_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <bfmc_msgs/msg/object_array.hpp>

#include "object_detection_cpp/config_loader.hpp"
#include "object_detection_cpp/trt_engine.hpp"
#include "object_detection_cpp/postprocess.hpp"
#include "object_detection_cpp/health_monitor.hpp"

namespace object_detection_cpp
{

class ObjectDetectionNode : public rclcpp::Node
{
public:
  explicit ObjectDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ObjectDetectionNode() override;

private:
  void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void depth_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void camera_info_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void run_pipeline(const sensor_msgs::msg::Image & rgb);

  // Pinned staging buffer for the host-side BGR/RGB image. Resized lazily.
  void ensure_host_input_capacity_(size_t bytes);

  // CUDA last-error check
  void check_cuda_(cudaError_t err, const char * where);

  DetectionConfig config_;
  std::unique_ptr<TrtEngine> engine_;
  std::unique_ptr<HealthMonitor> health_;

  // Subscriptions / publisher
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<bfmc_msgs::msg::ObjectArray>::SharedPtr pub_;

  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr aux_cb_group_;

  // Camera intrinsics (cached once)
  std::atomic<bool> intrinsics_ready_{false};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

  // Latest depth — protected by mutex. Pointer swap, no copy.
  std::mutex depth_mtx_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;

  // Pinned host buffer for the source RGB/BGR image (H2D source).
  uint8_t * h_image_pinned_{nullptr};
  size_t h_image_bytes_{0};

  // Persistent device staging buffer for the source image.
  uint8_t * d_image_buf_{nullptr};
  size_t d_image_bytes_{0};

  // Reusable scratch for decoded detections (avoid per-frame allocation).
  std::vector<Detection> detections_scratch_;

  // Reusable pre-allocated outgoing message
  std::unique_ptr<bfmc_msgs::msg::ObjectArray> out_msg_;

  // Stale-frame protection (parameter "stale_threshold_s", default 0.1 s).
  // Setting to 0.0 disables the check and restores legacy behavior verbatim.
  double stale_threshold_s_{0.1};
  std::uint64_t dropped_stale_{0};

  // Race-day P4: minimum period between processed frames. With the camera at
  // 15 Hz and min_period_s_=0.125, the detector caps itself at 8 Hz —
  // ~halves TRT load. Set to 0.0 to disable. Skipped frames return early
  // before TRT inference.
  double min_period_s_{0.125};
  rclcpp::Time last_processed_t_{0, 0, RCL_ROS_TIME};
  std::uint64_t dropped_throttle_{0};
};

}  // namespace object_detection_cpp

#endif  // OBJECT_DETECTION_CPP__OBJECT_DETECTION_NODE_HPP_
