// =============================================================================
// object_detection_node.cpp
// BFMC 2026 — Ultra low-latency YOLOv8 detection (TensorRT 10, fused CUDA
// preprocess). Drop-in replacement for the original Python node.
// =============================================================================

#include "object_detection_cpp/object_detection_node.hpp"
#include "object_detection_cpp/preprocess_cuda.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/qos.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace object_detection_cpp
{

namespace
{
// ---------------------------------------------------------------------------
// Depth helpers (preserve original Python semantics exactly)
// ---------------------------------------------------------------------------

// Returns true and fills *out_z if a valid median is found, otherwise false.
bool extract_depth_patch_median(
  const sensor_msgs::msg::Image & depth,
  float x1, float y1, float x2, float y2,
  float * out_z)
{
  const int h = static_cast<int>(depth.height);
  const int w = static_cast<int>(depth.width);
  if (h <= 0 || w <= 0 || depth.data.empty()) {
    return false;
  }

  const float bw = x2 - x1;
  const float bh = y2 - y1;

  int cx1 = std::max(0, static_cast<int>(x1 + bw * 0.4f));
  int cx2 = std::min(w, static_cast<int>(x2 - bw * 0.4f));
  int cy1 = std::max(0, static_cast<int>(y1 + bh * 0.4f));
  int cy2 = std::min(h, static_cast<int>(y2 - bh * 0.4f));

  if (cx2 <= cx1 || cy2 <= cy1) {
    return false;
  }

  std::vector<float> valid;
  valid.reserve(static_cast<size_t>((cy2 - cy1) * (cx2 - cx1)));

  const bool is_uint16 = (depth.encoding == "16UC1" || depth.encoding == "mono16");
  const size_t row_step = depth.step;

  if (is_uint16) {
    for (int y = cy1; y < cy2; ++y) {
      const uint16_t * row = reinterpret_cast<const uint16_t *>(
        depth.data.data() + y * row_step);
      for (int x = cx1; x < cx2; ++x) {
        const float z = static_cast<float>(row[x]) * 0.001f;
        if (z > 0.05f && z < 5.0f) {
          valid.push_back(z);
        }
      }
    }
  } else {
    // assume 32FC1 already in meters
    for (int y = cy1; y < cy2; ++y) {
      const float * row = reinterpret_cast<const float *>(
        depth.data.data() + y * row_step);
      for (int x = cx1; x < cx2; ++x) {
        const float z = row[x];
        if (std::isfinite(z) && z > 0.05f && z < 5.0f) {
          valid.push_back(z);
        }
      }
    }
  }

  if (valid.empty()) {
    return false;
  }

  // Median via nth_element (O(n))
  const size_t mid = valid.size() / 2;
  std::nth_element(valid.begin(), valid.begin() + mid, valid.end());
  *out_z = valid[mid];
  return true;
}

inline void pixel_to_camera(
  float u, float v, float z, double fx, double fy, double cx, double cy,
  float * x, float * y, float * z_out)
{
  *x = static_cast<float>((u - cx) * z / fx);
  *y = static_cast<float>((v - cy) * z / fy);
  *z_out = z;
}

inline void camera_to_vehicle(
  float x_c, float y_c, float z_c,
  float * vx, float * vy, float * vz)
{
  *vx =  z_c;
  *vy = -x_c;
  *vz = -y_c;
}

}  // namespace

// ---------------------------------------------------------------------------
// Node construction
// ---------------------------------------------------------------------------

ObjectDetectionNode::ObjectDetectionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("object_detection_node", options)
{
  RCLCPP_INFO(get_logger(), "ObjectDetectionNode (C++ / TRT %d) initializing",
              NV_TENSORRT_MAJOR);

  // Resolve config path: param > env > package default.
  declare_parameter<std::string>("config_path", "");
  const std::string param_path = get_parameter("config_path").as_string();

  // Stale-frame protection. Default 0.1 s matches the rest of the perception
  // stack; set to 0.0 to disable and restore the legacy always-run behavior.
  declare_parameter<double>("stale_threshold_s", 0.1);
  stale_threshold_s_ = get_parameter("stale_threshold_s").as_double();
  declare_parameter<double>("min_period_s", min_period_s_);
  min_period_s_ = get_parameter("min_period_s").as_double();

  std::string config_path = param_path;
  if (config_path.empty()) {
    if (const char * env = std::getenv("OBJDET_CONFIG_PATH")) {
      config_path = env;
    }
  }
  if (config_path.empty()) {
    // Resolve the canonical YAML from the installed share tree of this package.
    config_path = ament_index_cpp::get_package_share_directory("object_detection_cpp")
                  + "/config/object_detection.yaml";
  }

  config_ = load_config(config_path);
  RCLCPP_INFO(get_logger(), "Loaded config %s (%zu classes)",
              config_path.c_str(), config_.classes.size());

  // Operator-supplied engine_path parameter overrides the YAML value, which
  // lets the same YAML ship to different machines without editing.
  declare_parameter<std::string>("engine_path", "");
  const std::string engine_override = get_parameter("engine_path").as_string();
  if (!engine_override.empty()) {
    config_.model.engine_path = engine_override;
  }
  if (config_.model.engine_path.empty()) {
    throw std::runtime_error(
      "object_detection_node: engine_path must be set (either in the YAML's "
      "model.engine_path field or via the engine_path node parameter).");
  }

  // Build the TensorRT engine wrapper.
  engine_ = std::make_unique<TrtEngine>(
    config_.model.engine_path,
    config_.model.input_height,
    config_.model.input_width);

  RCLCPP_INFO(get_logger(),
              "Engine loaded: %s (output channels=%d, predictions=%d)",
              config_.model.engine_path.c_str(),
              engine_->output_channels(),
              engine_->output_predictions());

  // Health monitor
  health_ = std::make_unique<HealthMonitor>(
    get_logger(), config_.runtime.publish_rate, config_.runtime.max_log_rate_hz);

  // Pre-allocate scratch + output message.
  detections_scratch_.reserve(256);
  out_msg_ = std::make_unique<bfmc_msgs::msg::ObjectArray>();
  out_msg_->objects.reserve(64);

  // Warmup TRT before subscriptions go live so the first real frame is fast.
  engine_->warmup(config_.runtime.warmup_runs);
  RCLCPP_INFO(get_logger(), "TensorRT warmup complete (%d runs)",
              config_.runtime.warmup_runs);

  // --------------------------------------------------------------------------
  // ROS QoS:
  //   - Subscriptions to RealSense image / depth / camera_info: BEST_EFFORT.
  //     docker/realsense_params.yaml explicitly sets color_info_qos and
  //     depth_info_qos to SENSOR_DATA (BE). RGB/depth image streams default
  //     to RELIABLE in realsense2_camera, but BE on the subscriber side is
  //     compatible with both — and matches image_preprocessor_cpp +
  //     free_space_cpp, which already subscribe BE. This unifies the
  //     perception stack on a single, drop-friendly QoS contract.
  //
  //     PRIOR BUG (Phase 3 freeze audit): subscribing RELIABLE meant
  //     camera_info from the BE-publishing RealSense never reached us, so
  //     intrinsics_ready_ stayed false and ObjectArray was published
  //     without 3D positions for the whole session (tracker silently
  //     received 0,0,0 positions or no objects).
  //
  //   - Publishers: RELIABLE on /perception/objects (downstream tracker
  //     and planners can't tolerate dropped detections).
  // --------------------------------------------------------------------------
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto pub_qos    = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

  // Two callback groups so depth + intrinsics keep updating while the heavy
  // RGB callback is busy with TRT inference.
  image_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  aux_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions image_opts;
  image_opts.callback_group = image_cb_group_;
  rclcpp::SubscriptionOptions aux_opts;
  aux_opts.callback_group = aux_cb_group_;

  pub_ = create_publisher<bfmc_msgs::msg::ObjectArray>(
    config_.topics.publish, pub_qos);

  rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
    config_.topics.subscribe, sensor_qos,
    std::bind(&ObjectDetectionNode::image_callback, this, std::placeholders::_1),
    image_opts);

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    config_.topics.depth, sensor_qos,
    std::bind(&ObjectDetectionNode::depth_callback, this, std::placeholders::_1),
    aux_opts);

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.topics.camera_info, sensor_qos,
    std::bind(&ObjectDetectionNode::camera_info_callback, this, std::placeholders::_1),
    aux_opts);

  health_->start();
  RCLCPP_INFO(get_logger(), "ObjectDetectionNode READY (callback-driven)");
}

ObjectDetectionNode::~ObjectDetectionNode()
{
  if (h_image_pinned_) {
    cudaFreeHost(h_image_pinned_);
    h_image_pinned_ = nullptr;
  }
  if (d_image_buf_) {
    cudaFree(d_image_buf_);
    d_image_buf_ = nullptr;
  }
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void ObjectDetectionNode::camera_info_callback(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (intrinsics_ready_.load(std::memory_order_acquire)) {
    return;
  }
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  intrinsics_ready_.store(true, std::memory_order_release);
  RCLCPP_INFO(get_logger(),
              "Camera intrinsics cached: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
              fx_, fy_, cx_, cy_);
}

void ObjectDetectionNode::depth_callback(
  sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(depth_mtx_);
  latest_depth_ = std::move(msg);
}

void ObjectDetectionNode::image_callback(
  sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!intrinsics_ready_.load(std::memory_order_acquire)) {
    // Wait for camera_info before publishing positions.
    return;
  }

  // Race-day P4: rate limiter. Camera publishes at 15 Hz; we cap the detector
  // at 1 / min_period_s_ (default 8 Hz). Skipped frames return before TRT.
  if (min_period_s_ > 0.0) {
    const rclcpp::Time now_t = now();
    if (last_processed_t_.nanoseconds() != 0 &&
        (now_t - last_processed_t_).seconds() < min_period_s_) {
      ++dropped_throttle_;
      return;
    }
    last_processed_t_ = now_t;
  }

  // Stale-frame protection. Drops a too-old frame *before* TRT runs so a
  // backpressured pipeline can catch up instead of compounding spike latency.
  // Gated by stale_threshold_s_; set to 0.0 to disable.
  if (stale_threshold_s_ > 0.0) {
    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() != 0) {
      const rclcpp::Time now_t = now();
      const double age = (now_t - stamp).seconds();
      if (age > stale_threshold_s_) {
        ++dropped_stale_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "object_detection dropping stale frame (age %.0f ms > %.0f ms, dropped=%lu)",
          age * 1000.0, stale_threshold_s_ * 1000.0,
          static_cast<unsigned long>(dropped_stale_));
        return;
      }
    }
  }

  try {
    run_pipeline(*msg);
  } catch (const std::exception & e) {
    health_->record_error(e.what());
  }
}

// ---------------------------------------------------------------------------
// Hot path
// ---------------------------------------------------------------------------

void ObjectDetectionNode::ensure_host_input_capacity_(size_t bytes)
{
  if (bytes <= h_image_bytes_ && h_image_pinned_ != nullptr) {
    return;
  }
  if (h_image_pinned_) {
    cudaFreeHost(h_image_pinned_);
    h_image_pinned_ = nullptr;
  }
  void * p = nullptr;
  const auto err = cudaHostAlloc(&p, bytes, cudaHostAllocDefault);
  check_cuda_(err, "cudaHostAlloc(h_image_pinned_)");
  h_image_pinned_ = static_cast<uint8_t *>(p);
  h_image_bytes_ = bytes;
}

void ObjectDetectionNode::check_cuda_(cudaError_t err, const char * where)
{
  if (err != cudaSuccess) {
    std::ostringstream s;
    s << "CUDA error at " << where << ": " << cudaGetErrorString(err);
    throw std::runtime_error(s.str());
  }
}

void ObjectDetectionNode::run_pipeline(const sensor_msgs::msg::Image & rgb)
{
  const auto t0 = std::chrono::steady_clock::now();

  // Encoding detection: RealSense default is rgb8, but the original Python
  // forced bgr8 via cv_bridge. We accept either and tell the fused kernel
  // whether to swap channels.
  const bool is_bgr = (rgb.encoding == "bgr8");
  const bool is_rgb = (rgb.encoding == "rgb8");
  if (!is_bgr && !is_rgb) {
    std::ostringstream s;
    s << "unsupported color encoding: " << rgb.encoding;
    throw std::runtime_error(s.str());
  }

  const int src_h = static_cast<int>(rgb.height);
  const int src_w = static_cast<int>(rgb.width);
  const int src_step = static_cast<int>(rgb.step);
  const size_t src_bytes = static_cast<size_t>(src_step) * static_cast<size_t>(src_h);

  // Pinned host buffer: enables truly async cudaMemcpy and lets the GPU
  // overlap H2D with the next callback's CPU work.
  ensure_host_input_capacity_(src_bytes);
  std::memcpy(h_image_pinned_, rgb.data.data(), src_bytes);

  // Persistent device staging buffer (re-allocated only if frame grows).
  if (src_bytes > d_image_bytes_) {
    if (d_image_buf_) {
      cudaFree(d_image_buf_);
      d_image_buf_ = nullptr;
    }
    void * p = nullptr;
    check_cuda_(cudaMalloc(&p, src_bytes), "cudaMalloc(d_image_buf)");
    d_image_buf_ = static_cast<uint8_t *>(p);
    d_image_bytes_ = src_bytes;
  }

  // -------------------- GPU pipeline (single stream) -------------------------
  // 1) async H2D from pinned host -> device staging
  check_cuda_(cudaMemcpyAsync(d_image_buf_, h_image_pinned_, src_bytes,
                              cudaMemcpyHostToDevice, engine_->stream()),
              "H2D image");

  // 2) fused resize + (BGR<->RGB) + /255 + NCHW scatter into the TRT input
  check_cuda_(launch_preprocess(
                d_image_buf_, src_h, src_w, src_step, is_bgr,
                engine_->device_input(),
                engine_->input_h(), engine_->input_w(),
                engine_->stream()),
              "preprocess kernel");

  // 3) TRT enqueue + D2H of output
  engine_->enqueue();

  // 4) wait for the result (a few ms on Orin Nano)
  engine_->sync();

  // ----------------------- Decode + NMS on CPU -------------------------------
  decode_yolov8(
    engine_->host_output(),
    static_cast<int>(config_.classes.size()),
    engine_->output_predictions(),
    config_.thresholds.conf,
    config_.thresholds.nms,
    src_h, src_w,
    engine_->input_h(), engine_->input_w(),
    detections_scratch_);

  // ----------------------- Depth projection + publish ------------------------
  // Grab latest depth pointer atomically (one shared_ptr copy = atomic ref-inc).
  sensor_msgs::msg::Image::ConstSharedPtr depth_snap;
  {
    std::lock_guard<std::mutex> lk(depth_mtx_);
    depth_snap = latest_depth_;
  }

  out_msg_->objects.clear();
  out_msg_->stamp = rgb.header.stamp;

  for (const Detection & det : detections_scratch_) {
    const float x1 = det.bbox[0];
    const float y1 = det.bbox[1];
    const float x2 = det.bbox[2];
    const float y2 = det.bbox[3];

    const float u = (x1 + x2) * 0.5f;
    const float v = (y1 + y2) * 0.5f;

    // Default fallback (matches original Python behavior when no valid patch).
    float z = 1.5f;
    if (depth_snap) {
      float zz = 0.0f;
      if (extract_depth_patch_median(*depth_snap, x1, y1, x2, y2, &zz)) {
        z = zz;
      }
    }

    float cam_x, cam_y, cam_z;
    pixel_to_camera(u, v, z, fx_, fy_, cx_, cy_, &cam_x, &cam_y, &cam_z);

    float vx, vy, vz;
    camera_to_vehicle(cam_x, cam_y, cam_z, &vx, &vy, &vz);

    bfmc_msgs::msg::DetectedObject obj;
    obj.id = static_cast<uint8_t>(det.class_id);
    obj.class_name = (det.class_id >= 0 &&
                      static_cast<size_t>(det.class_id) < config_.classes.size())
                       ? config_.classes[static_cast<size_t>(det.class_id)]
                       : std::string("class_") + std::to_string(det.class_id);
    obj.confidence = det.confidence;
    obj.position.x = vx;
    obj.position.y = vy;
    obj.position.z = vz;
    out_msg_->objects.push_back(std::move(obj));
  }

  // Publish (clone via copy ctor — ObjectArray contents are small).
  pub_->publish(*out_msg_);

  // ----------------------- Health bookkeeping --------------------------------
  const auto t1 = std::chrono::steady_clock::now();
  const double latency_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  health_->record_inference(latency_ms);
  health_->log_periodic_summary();
}

}  // namespace object_detection_cpp
