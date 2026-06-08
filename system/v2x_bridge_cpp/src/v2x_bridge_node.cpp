#include "v2x_bridge_cpp/v2x_bridge_node.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>

namespace v2x_bridge_cpp
{

namespace
{
double yaw_from_quat(double qz, double qw) { return 2.0 * std::atan2(qz, qw); }
}  // namespace

V2xBridgeNode::V2xBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v2x_bridge_node", options)
{
  // ---- Parameters --------------------------------------------------------
  udp_port_   = declare_parameter<int>("udp_port",   udp_port_);
  tcp_host_   = declare_parameter<std::string>("tcp_host", tcp_host_);
  tcp_port_   = declare_parameter<int>("tcp_port",   tcp_port_);
  locsys_id_  = declare_parameter<int>("locsys_id",  locsys_id_);
  tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", tx_rate_hz_);
  enable_tcp_ = declare_parameter<bool>("enable_tcp", enable_tcp_);

  // BFMC server gate: frequency must be in [0.1, 5.0] per protocol spec.
  tx_rate_hz_ = std::clamp(tx_rate_hz_, 0.1, 5.0);

  // ---- ROS interfaces ----------------------------------------------------
  auto rel_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  pub_semaphores_ = create_publisher<std_msgs::msg::String>("/v2x/semaphores", rel_qos);
  pub_cars_       = create_publisher<std_msgs::msg::String>("/v2x/cars",       rel_qos);
  sub_state_      = create_subscription<bfmc_msgs::msg::VehicleState>(
    "/vehicle/state", rel_qos,
    std::bind(&V2xBridgeNode::on_vehicle_state, this, std::placeholders::_1));

  // ---- Worker threads ----------------------------------------------------
  udp_thread_ = std::thread(&V2xBridgeNode::udp_loop_, this);
  if (enable_tcp_) {
    tcp_thread_ = std::thread(&V2xBridgeNode::tcp_loop_, this);
  }

  RCLCPP_INFO(get_logger(),
    "v2x_bridge: UDP listen=:%d  TCP=%s:%d locsys_id=%d freq=%.2fHz enabled=%s",
    udp_port_, tcp_host_.c_str(), tcp_port_,
    locsys_id_, tx_rate_hz_, enable_tcp_ ? "true" : "false");
}

V2xBridgeNode::~V2xBridgeNode()
{
  running_.store(false);
  // Give blocking sockets a moment to notice the flag; close-from-other-thread
  // would also work but is harder to get right portably. recvfrom uses
  // SO_RCVTIMEO so it'll wake up within a second.
  if (udp_thread_.joinable()) udp_thread_.join();
  if (tcp_thread_.joinable()) tcp_thread_.join();
}

/* ---------------- State subscription (TCP upload feed) ---------------- */

void V2xBridgeNode::on_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(tx_mtx_);
  last_x_     = msg->pose.position.x;
  last_y_     = msg->pose.position.y;
  last_yaw_   = yaw_from_quat(msg->pose.orientation.z, msg->pose.orientation.w);
  last_speed_ = msg->speed;
  last_state_stamp_ = this->now().seconds();
  have_state_ = true;
}

/* ---------------- Socket helpers ---------------- */

bool V2xBridgeNode::open_udp_socket_(int port, int & out_fd, std::string & err)
{
  out_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (out_fd < 0) { err = "socket() failed"; return false; }

  int yes = 1;
  ::setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // 1-second recv timeout so the loop can poll `running_`.
  struct timeval tv{1, 0};
  ::setsockopt(out_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(static_cast<uint16_t>(port));
  if (::bind(out_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = std::string("bind() failed: ") + std::strerror(errno);
    ::close(out_fd);
    out_fd = -1;
    return false;
  }
  return true;
}

bool V2xBridgeNode::open_tcp_socket_(const std::string & host, int port,
                                     int & out_fd, std::string & err)
{
  out_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (out_fd < 0) { err = "socket() failed"; return false; }

  // 2-second connect / send timeout — server unreachable should not stall us.
  struct timeval tv{2, 0};
  ::setsockopt(out_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  ::setsockopt(out_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    err = "invalid TCP host (must be IPv4 literal)";
    ::close(out_fd);
    out_fd = -1;
    return false;
  }
  if (::connect(out_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = std::string("connect() failed: ") + std::strerror(errno);
    ::close(out_fd);
    out_fd = -1;
    return false;
  }
  return true;
}

bool V2xBridgeNode::send_all_(int fd, const std::string & payload)
{
  std::size_t sent = 0;
  while (sent < payload.size()) {
    const ssize_t n = ::send(fd, payload.data() + sent, payload.size() - sent, 0);
    if (n <= 0) return false;
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

/* ---------------- UDP receive loop ---------------- */

void V2xBridgeNode::udp_loop_()
{
  int fd = -1;
  std::string err;
  if (!open_udp_socket_(udp_port_, fd, err)) {
    RCLCPP_ERROR(get_logger(), "UDP open failed (port %d): %s — receive disabled",
                 udp_port_, err.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "UDP listening on port %d", udp_port_);

  std::vector<char> buf(2048);
  while (running_.load()) {
    sockaddr_in src{};
    socklen_t   src_len = sizeof(src);
    const ssize_t n = ::recvfrom(fd, buf.data(), buf.size() - 1, 0,
                                 reinterpret_cast<sockaddr *>(&src), &src_len);
    if (n <= 0) {
      // EAGAIN/EWOULDBLOCK on timeout — re-check running_ and loop.
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
      RCLCPP_WARN(get_logger(), "UDP recv error: %s", std::strerror(errno));
      continue;
    }
    buf[n] = '\0';
    const std::string raw(buf.data(), static_cast<std::size_t>(n));

    // Inline device-classification: scan once for "device":"semaphore" /
    // "device":"car". Forward to the matching topic; payload stays raw JSON.
    std_msgs::msg::String out;
    out.data = raw;
    if (raw.find("\"device\":\"semaphore\"") != std::string::npos) {
      pub_semaphores_->publish(out);
    } else if (raw.find("\"device\":\"car\"") != std::string::npos) {
      pub_cars_->publish(out);
    } else {
      RCLCPP_DEBUG(get_logger(), "UDP unknown payload: %s", raw.c_str());
    }
  }
  if (fd >= 0) ::close(fd);
}

/* ---------------- TCP send loop ---------------- */

void V2xBridgeNode::tcp_loop_()
{
  // Reconnect outer loop. Inner loop sends periodic location updates.
  while (running_.load()) {
    if (tcp_host_.empty()) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    int fd = -1;
    std::string err;
    if (!open_tcp_socket_(tcp_host_, tcp_port_, fd, err)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TCP connect to %s:%d failed: %s — retrying",
        tcp_host_.c_str(), tcp_port_, err.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    RCLCPP_INFO(get_logger(), "TCP connected to %s:%d", tcp_host_.c_str(), tcp_port_);

    // Subscribe message per tcpClient.py protocol.
    {
      std::ostringstream os;
      os << "{\"reqORinfo\":\"info\",\"type\":\"locIDsub\","
         << "\"locID\":" << locsys_id_ << ","
         << "\"freq\":"  << tx_rate_hz_ << "}";
      if (!send_all_(fd, os.str())) {
        RCLCPP_WARN(get_logger(), "TCP subscribe send failed — reconnecting");
        ::close(fd);
        continue;
      }
    }

    const auto period_us =
      std::chrono::microseconds(static_cast<long>(1e6 / tx_rate_hz_));
    auto next_tx = std::chrono::steady_clock::now();
    while (running_.load()) {
      next_tx += period_us;

      // Snapshot under the lock — keep critical section tiny.
      double x = 0, y = 0, yaw = 0, sp = 0, ts = 0;
      bool have = false;
      {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        have = have_state_;
        x = last_x_; y = last_y_; yaw = last_yaw_;
        sp = last_speed_; ts = last_state_stamp_;
      }
      if (have) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(4);
        os << "{\"type\":\"location\","
           << "\"id\":" << locsys_id_ << ","
           << "\"x\":"  << x   << ","
           << "\"y\":"  << y   << ","
           << "\"yaw\":"<< yaw << ","
           << "\"speed\":" << sp << ","
           << "\"timestamp\":" << ts << "}";
        if (!send_all_(fd, os.str())) {
          RCLCPP_WARN(get_logger(), "TCP send failed — reconnecting");
          break;
        }
      }
      std::this_thread::sleep_until(next_tx);
    }
    ::close(fd);
  }
}

}  // namespace v2x_bridge_cpp
