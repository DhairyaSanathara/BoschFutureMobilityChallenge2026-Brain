#ifndef V2X_BRIDGE_CPP__V2X_BRIDGE_NODE_HPP_
#define V2X_BRIDGE_CPP__V2X_BRIDGE_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bfmc_msgs/msg/vehicle_state.hpp"

namespace v2x_bridge_cpp
{

// V2X bridge — UDP receive + TCP location uplink, both on dedicated worker
// threads so the ROS executor stays unblocked.
//
// UDP wire format (JSON, one object per datagram):
//   {"device":"semaphore","id":<id>,"state":<>,"x":..,"y":..}
//   {"device":"car",      "id":<id>,                 "x":..,"y":..}
// Both are forwarded verbatim as std_msgs/String on /v2x/semaphores or
// /v2x/cars respectively — downstream FSM parses them with a tiny inline
// JSON extractor (no nlohmann/json dependency).
//
// TCP wire format (JSON line-framed):
//   first send the subscribe message:
//     {"reqORinfo":"info","type":"locIDsub","locID":<id>,"freq":<hz>}
//   then send periodic location updates at `tx_rate_hz`:
//     {"type":"location","id":<id>,"x":..,"y":..,"yaw":..,"speed":..,
//      "timestamp":..}
// We do NOT consume the TCP receive direction in this bridge (the server's
// echo of our own location is not actionable for our autonomy).
class V2xBridgeNode : public rclcpp::Node
{
public:
  explicit V2xBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~V2xBridgeNode() override;

private:
  void on_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg);

  // UDP receive thread body.
  void udp_loop_();
  // TCP send thread body (with auto-reconnect).
  void tcp_loop_();

  // Helpers shared by both threads.
  static bool open_udp_socket_(int port, int & out_fd, std::string & err);
  static bool open_tcp_socket_(const std::string & host, int port,
                               int & out_fd, std::string & err);
  static bool send_all_(int fd, const std::string & payload);

  // Subs / pubs.
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_semaphores_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_cars_;

  // Threads.
  std::thread udp_thread_;
  std::thread tcp_thread_;
  std::atomic<bool> running_{true};

  // TCP send state — protected by tx_mtx_.
  std::mutex tx_mtx_;
  bool   have_state_{false};
  double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0}, last_speed_{0.0};
  double last_state_stamp_{0.0};

  // Parameters.
  int         udp_port_{5007};
  std::string tcp_host_;             // empty ⇒ TCP loop idles (race-day setup)
  int         tcp_port_{9000};
  int         locsys_id_{0};
  double      tx_rate_hz_{2.0};      // BFMC server expects 0.1..5.0
  bool        enable_tcp_{false};    // explicit opt-in, default off
};

}  // namespace v2x_bridge_cpp

#endif  // V2X_BRIDGE_CPP__V2X_BRIDGE_NODE_HPP_
