#ifndef EKF_LOCALIZATION__LOCALIZATION_CLIENT_NODE_HPP_
#define EKF_LOCALIZATION__LOCALIZATION_CLIENT_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace bfmc_fusion
{

// TCP-JSON client for the BFMC localization server.
//   - One persistent socket in a child thread (non-blocking via SO_RCVTIMEO).
//   - Auto-reconnects with a configurable backoff if the server drops.
//   - Parses {"x": <num>, "y": <num>, ...} payloads (one per object/line);
//     other fields are ignored.
//   - Publishes geometry_msgs/PointStamped on /localization/raw_position
//     (frame_id="map" by default).
class LocalizationClientNode : public rclcpp::Node
{
public:
  explicit LocalizationClientNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationClientNode() override;

private:
  void io_loop();
  void handle_payload(const std::string & json_chunk);

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;

  std::thread io_thread_;
  std::atomic<bool> running_{true};

  // Parameters
  std::string server_host_;
  int server_port_{0};
  std::string frame_id_;
  double reconnect_delay_s_{1.0};
  double recv_timeout_s_{2.0};

  // Diagnostics
  std::atomic<uint64_t> msg_count_{0};
  std::atomic<uint64_t> parse_errors_{0};
};

}  // namespace bfmc_fusion

#endif  // EKF_LOCALIZATION__LOCALIZATION_CLIENT_NODE_HPP_
