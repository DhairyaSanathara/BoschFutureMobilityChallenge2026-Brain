#include "free_space_cpp/free_space_node.hpp"

#include <cmath>
#include <cstring>
#include <arm_neon.h>

#include <rclcpp_components/register_node_macro.hpp>

namespace free_space_cpp
{

// 8-lane NEON EMA + classification. Previous version loaded 8 lanes per
// iteration but only consumed the low 4, halving throughput; this version
// processes all 8 lanes per step.
inline void stabilise_grid_neon(
    const uint8_t* current, float* ema, uint8_t* publish, uint32_t size, float alpha)
{
    const float32x4_t v_alpha = vdupq_n_f32(alpha);
    const float32x4_t v_inv = vdupq_n_f32(1.0f - alpha);

    uint32_t i = 0;
    for (; i + 8 <= size; i += 8)
    {
        // Load 8 uint8 lanes and widen to two float32x4 vectors.
        const uint8x8_t cur8 = vld1_u8(current + i);
        const uint16x8_t cur16 = vmovl_u8(cur8);
        const float32x4_t cf_lo = vcvtq_f32_u32(vmovl_u16(vget_low_u16(cur16)));
        const float32x4_t cf_hi = vcvtq_f32_u32(vmovl_u16(vget_high_u16(cur16)));

        // EMA: new = (1-a)*prev + a*current.
        float32x4_t ema_lo = vld1q_f32(ema + i);
        float32x4_t ema_hi = vld1q_f32(ema + i + 4);
        ema_lo = vmlaq_f32(vmulq_f32(v_inv, ema_lo), v_alpha, cf_lo);
        ema_hi = vmlaq_f32(vmulq_f32(v_inv, ema_hi), v_alpha, cf_hi);
        vst1q_f32(ema + i, ema_lo);
        vst1q_f32(ema + i + 4, ema_hi);

        // Classify each lane: <30 -> 0 (free), >60 -> 100 (occupied), else 255 (unknown).
        float tmp_lo[4], tmp_hi[4];
        vst1q_f32(tmp_lo, ema_lo);
        vst1q_f32(tmp_hi, ema_hi);
        for (int j = 0; j < 4; ++j) {
            const float v = tmp_lo[j];
            publish[i + j]     = (v < 30.f) ? 0 : (v > 60.f ? 100 : 255);
            const float w = tmp_hi[j];
            publish[i + 4 + j] = (w < 30.f) ? 0 : (w > 60.f ? 100 : 255);
        }
    }
    for (; i < size; ++i) {
        const float val = alpha * current[i] + (1.f - alpha) * ema[i];
        ema[i] = val;
        publish[i] = val < 30.f ? 0 : (val > 60.f ? 100 : 255);
    }
}

FreeSpaceNode::FreeSpaceNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("free_space_node", options)
{
    declare_parameter("depth_min", 0.2);
    declare_parameter("depth_max", 3.0);
    declare_parameter("grid_resolution", 0.05);
    declare_parameter("grid_width_m", 2.0);
    declare_parameter("grid_length_m", 3.0);
    declare_parameter("ema_alpha", 0.4);
    declare_parameter("camera_height_m", 0.30);
    declare_parameter("camera_pitch_deg", 6.0);
    declare_parameter("ground_clearance_m", 0.06);
    declare_parameter("ground_tolerance_m", 0.04);
    declare_parameter("pixel_stride", 4);
    declare_parameter("watchdog_timeout_s", 0.5);
    declare_parameter("stale_threshold_s", 0.1);
    declare_parameter("min_period_s", min_period_s_);

    steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Two reentrant groups so the watchdog tick is never blocked by an
    // in-flight depth callback (and vice-versa).
    depth_cb_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_cb_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

FreeSpaceNode::CallbackReturn FreeSpaceNode::on_configure(const rclcpp_lifecycle::State &)
{
    depth_min_ = get_parameter("depth_min").as_double();
    depth_max_ = get_parameter("depth_max").as_double();
    grid_resolution_ = get_parameter("grid_resolution").as_double();
    grid_width_m_ = get_parameter("grid_width_m").as_double();
    grid_length_m_ = get_parameter("grid_length_m").as_double();
    ema_alpha_ = get_parameter("ema_alpha").as_double();
    camera_height_m_ = get_parameter("camera_height_m").as_double();
    ground_clearance_m_ = get_parameter("ground_clearance_m").as_double();
    ground_tolerance_m_ = get_parameter("ground_tolerance_m").as_double();
    pixel_stride_ = get_parameter("pixel_stride").as_int();
    watchdog_timeout_s_ = get_parameter("watchdog_timeout_s").as_double();
    stale_threshold_s_ = get_parameter("stale_threshold_s").as_double();
    min_period_s_      = get_parameter("min_period_s").as_double();
    last_diag_log_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    camera_pitch_rad_ = get_parameter("camera_pitch_deg").as_double() * M_PI / 180.0;

    grid_cols_ = std::ceil(grid_width_m_ / grid_resolution_);
    grid_rows_ = std::ceil(grid_length_m_ / grid_resolution_);
    grid_size_ = grid_cols_ * grid_rows_;

    current_grid_.resize(grid_size_, 255);
    ema_grid_.resize(grid_size_, 255.f);
    publish_grid_.resize(grid_size_, 255);

    // SensorDataQoS depth 1, explicit BEST_EFFORT: newest grid wins, matches
    // downstream BEST_EFFORT subscribers (behavior_planner).
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    grid_pub_ = create_publisher<bfmc_msgs::msg::FreeSpace>("/perception/free_space", qos);

    // camera_info from RealSense is published BEST_EFFORT; using SensorDataQoS
    // ensures we never miss the first intrinsics due to a QoS handshake failure.
    rclcpp::SubscriptionOptions info_opts;
    info_opts.callback_group = depth_cb_group_;
    auto info_qos = rclcpp::SensorDataQoS().best_effort();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info", info_qos,
        std::bind(&FreeSpaceNode::camera_info_callback, this, std::placeholders::_1),
        info_opts);

    return CallbackReturn::SUCCESS;
}

FreeSpaceNode::CallbackReturn FreeSpaceNode::on_activate(const rclcpp_lifecycle::State &)
{
    // RealSense publishes depth as a sensor stream (BEST_EFFORT). SensorDataQoS
    // with depth 1 keeps us synchronised to the newest frame under load.
    auto depth_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();

    rclcpp::SubscriptionOptions depth_opts;
    depth_opts.callback_group = depth_cb_group_;
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", depth_qos,
        std::bind(&FreeSpaceNode::depth_callback, this, std::placeholders::_1),
        depth_opts);

    grid_pub_->on_activate();
    last_depth_time_steady_ = steady_clock_->now();
    // Watchdog runs on its own group so a long depth callback can't starve it.
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FreeSpaceNode::watchdog_callback, this),
        watchdog_cb_group_);

    return CallbackReturn::SUCCESS;
}

FreeSpaceNode::CallbackReturn FreeSpaceNode::on_deactivate(const rclcpp_lifecycle::State &) { if (grid_pub_) grid_pub_->on_deactivate(); return CallbackReturn::SUCCESS; }
FreeSpaceNode::CallbackReturn FreeSpaceNode::on_cleanup(const rclcpp_lifecycle::State &) { depth_sub_.reset(); info_sub_.reset(); grid_pub_.reset(); watchdog_timer_.reset(); return CallbackReturn::SUCCESS; }
FreeSpaceNode::CallbackReturn FreeSpaceNode::on_shutdown(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }
FreeSpaceNode::CallbackReturn FreeSpaceNode::on_error(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

void FreeSpaceNode::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    // Always tick the watchdog clock, even if we drop the frame, so we don't
    // trip a false unknown-grid emission while frames are still arriving.
    last_depth_time_steady_ = steady_clock_->now();
    if (msg->data.empty() || !intrinsics_received_) return;

    // Race-day P4: rate limiter. Caps depth processing at 1/min_period_s_
    // (default 10 Hz) even when the camera publishes faster.
    if (min_period_s_ > 0.0) {
        const rclcpp::Time now_t2 = now();
        if (last_processed_t_.nanoseconds() != 0 &&
            (now_t2 - last_processed_t_).seconds() < min_period_s_) {
            ++dropped_throttle_;
            return;
        }
        last_processed_t_ = now_t2;
    }

    // Stale-frame protection (ROS-time based, sim-safe via clock interface).
    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    const rclcpp::Time now_t = now();
    if (stale_threshold_s_ > 0.0 &&
        stamp.nanoseconds() != 0 &&
        (now_t - stamp).seconds() > stale_threshold_s_)
    {
        ++dropped_stale_;
        auto throttle_clock = get_clock();
        RCLCPP_WARN_THROTTLE(get_logger(), *throttle_clock, 2000,
            "free_space dropping stale depth frame (age %.0f ms > %.0f ms)",
            (now_t - stamp).seconds() * 1000.0,
            stale_threshold_s_ * 1000.0);
        return;
    }

    depth_is_float_ = (msg->encoding == "32FC1");
    depth_width_ = msg->width;
    depth_height_ = msg->height;
    depth_step_ = msg->step;

    process_depth_to_grid(*msg);
    stabilise_grid();
    publish_grid();
    ++processed_frames_;

    if (last_diag_log_.nanoseconds() == 0 ||
        (now_t - last_diag_log_).seconds() > 5.0)
    {
        RCLCPP_INFO(get_logger(),
            "free_space: processed=%lu dropped_stale=%lu",
            static_cast<unsigned long>(processed_frames_),
            static_cast<unsigned long>(dropped_stale_));
        processed_frames_ = 0;
        dropped_stale_ = 0;
        last_diag_log_ = now_t;
    }
}

void FreeSpaceNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
    if (intrinsics_received_) return;
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    intrinsic_width_ = msg->width; intrinsic_height_ = msg->height;
    intrinsics_received_ = true;
}
void FreeSpaceNode::process_depth_to_grid(const sensor_msgs::msg::Image & msg)
{
    std::memset(current_grid_.data(), 255, grid_size_);

    const float cos_theta = std::cos(camera_pitch_rad_);
    const float sin_theta = std::sin(camera_pitch_rad_);
    const float h_cam = camera_height_m_;

    const float gc_threshold = ground_clearance_m_ - ground_tolerance_m_;
    const float res = grid_resolution_;
    const float half_w = grid_width_m_ * 0.5f;

    for (uint32_t v = 0; v < depth_height_; v += pixel_stride_)
    {
        const float vy = (static_cast<float>(v) - cy_) / fy_;
        const uint8_t * row_ptr = &msg.data[v * depth_step_];
        
        for (uint32_t u = 0; u < depth_width_; u += pixel_stride_)
        {
            float d;
            if (depth_is_float_) d = reinterpret_cast<const float*>(row_ptr)[u];
            else d = reinterpret_cast<const uint16_t*>(row_ptr)[u] * 0.001f;

            if (!std::isfinite(d) || d < depth_min_ || d > depth_max_) continue;

            // THE MISSING GROUND FILTER: Ignores the track floor so the car can drive!
            const float height_above_ground = h_cam - d * (vy * cos_theta + sin_theta);
            if (height_above_ground < gc_threshold) continue;

            const float x_c = (u - cx_) * d / fx_;
            const float y_c = vy * d;
            const float x_v = -y_c * sin_theta + d * cos_theta;
            const float y_v = -x_c;

            if (x_v < 0.f || x_v >= grid_length_m_) continue;
            const float y_shifted = y_v + half_w;
            if (y_shifted < 0.f || y_shifted >= grid_width_m_) continue;

            const uint32_t row = x_v / res;
            const uint32_t col = y_shifted / res;

            current_grid_[row * grid_cols_ + col] = 100;
        }
    }

    for (uint32_t col = 0; col < grid_cols_; ++col)
    {
        for (uint32_t row = 0; row < grid_rows_; ++row)
        {
            const uint32_t idx = row * grid_cols_ + col;
            if (current_grid_[idx] == 100) break;
            current_grid_[idx] = 0;
        }
    }
}
void FreeSpaceNode::stabilise_grid()
{
    stabilise_grid_neon(current_grid_.data(), ema_grid_.data(), publish_grid_.data(), grid_size_, ema_alpha_);
    ema_initialised_ = true;
}

void FreeSpaceNode::publish_grid()
{
    // One alloc, one copy: build the outgoing message directly from publish_grid_
    // rather than templating into pre_msg_ and copy-constructing.
    auto msg = std::make_unique<bfmc_msgs::msg::FreeSpace>();
    msg->width = grid_cols_;
    msg->height = grid_rows_;
    msg->resolution = grid_resolution_;
    msg->occupancy_grid.resize(grid_size_);
    std::memcpy(msg->occupancy_grid.data(), publish_grid_.data(), grid_size_);
    grid_pub_->publish(std::move(msg));
}

void FreeSpaceNode::watchdog_callback()
{
    if (!grid_pub_->is_activated()) return;
    if ((steady_clock_->now() - last_depth_time_steady_).seconds() > watchdog_timeout_s_) {
        publish_unknown_grid();
    }
}

void FreeSpaceNode::publish_unknown_grid()
{
    std::fill(publish_grid_.begin(), publish_grid_.end(), 255);
    publish_grid();
}

}  // namespace free_space_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(free_space_cpp::FreeSpaceNode)
