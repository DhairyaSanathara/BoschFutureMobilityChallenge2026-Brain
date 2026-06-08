#ifndef FREE_SPACE_CPP__FREE_SPACE_NODE_HPP_
#define FREE_SPACE_CPP__FREE_SPACE_NODE_HPP_

#include <vector>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <bfmc_msgs/msg/free_space.hpp>

namespace free_space_cpp
{

class FreeSpaceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit FreeSpaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FreeSpaceNode() override = default;

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
    void watchdog_callback();

    void process_depth_to_grid(const sensor_msgs::msg::Image & msg);
    void stabilise_grid();
    void publish_grid();
    void publish_unknown_grid();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp_lifecycle::LifecyclePublisher<bfmc_msgs::msg::FreeSpace>::SharedPtr grid_pub_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Separate callback groups so the watchdog timer can fire even while a
    // depth callback is still running (and vice-versa). Both reentrant so the
    // MT executor can dispatch concurrently.
    rclcpp::CallbackGroup::SharedPtr depth_cb_group_;
    rclcpp::CallbackGroup::SharedPtr watchdog_cb_group_;

    // Stale-frame protection
    double stale_threshold_s_{0.1};
    uint64_t dropped_stale_{0};
    uint64_t processed_frames_{0};
    rclcpp::Time last_diag_log_;

    // Race-day P4: depth-callback rate limiter. Camera = 15 Hz; with
    // min_period_s_ = 0.1 (= 10 Hz) we halve the projection workload.
    double min_period_s_{0.1};
    rclcpp::Time last_processed_t_{0, 0, RCL_ROS_TIME};
    uint64_t dropped_throttle_{0};

    rclcpp::Time last_depth_time_steady_;
    double watchdog_timeout_s_{0.5};

    bool intrinsics_received_{false};

    double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
    uint32_t intrinsic_width_{0}, intrinsic_height_{0};

    uint32_t depth_width_{0}, depth_height_{0}, depth_step_{0};
    bool depth_is_float_{false};

    double camera_height_m_{0.0};
    double camera_pitch_rad_{0.0};

    double depth_min_{0.2}, depth_max_{3.0};
    double grid_resolution_{0.05};
    double grid_width_m_{2.0}, grid_length_m_{3.0};
    double ema_alpha_{0.4};
    double ground_clearance_m_{0.06};
    double ground_tolerance_m_{0.04};

    int pixel_stride_{4};

    uint32_t grid_cols_{0}, grid_rows_{0}, grid_size_{0};

    std::vector<uint8_t> current_grid_;
    std::vector<float> ema_grid_;
    std::vector<uint8_t> publish_grid_;

    bool ema_initialised_{false};

    rclcpp::Clock::SharedPtr steady_clock_;
};

}

#endif
