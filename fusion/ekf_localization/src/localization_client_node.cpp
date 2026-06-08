#include "ekf_localization/localization_client_node.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace bfmc_fusion
{

namespace
{
// Minimal JSON field extractor for {"x": <num>, "y": <num>}.
// Avoids dragging nlohmann::json into this single tiny client.
// Returns true only if BOTH x and y parsed to finite doubles.
bool extract_xy(const std::string & s, double & x, double & y)
{
  auto get_num = [&](const char * key, double & out) -> bool {
    std::string needle = std::string("\"") + key + "\"";
    auto pos = s.find(needle);
    if (pos == std::string::npos) return false;
    pos = s.find(':', pos);
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
    if (pos >= s.size()) return false;
    char * end = nullptr;
    const char * begin = s.c_str() + pos;
    out = std::strtod(begin, &end);
    if (end == begin) return false;
    return std::isfinite(out);
  };
  return get_num("x", x) && get_num("y", y);
}
}  // namespace

LocalizationClientNode::LocalizationClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("localization_client_node", options)
{
  // Defaults are empty/zero — node is a no-op until the user configures a
  // real BFMC server endpoint. This avoids spamming connect attempts to a
  // placeholder address and ensures the rest of the stack does not depend
  // on this node existing.
  server_host_       = declare_parameter<std::string>("server_host", "");
  server_port_       = declare_parameter<int>("server_port", 0);
  frame_id_          = declare_parameter<std::string>("frame_id", "map");
  reconnect_delay_s_ = declare_parameter<double>("reconnect_delay_s", 1.0);
  recv_timeout_s_    = declare_parameter<double>("recv_timeout_s", 2.0);

  auto qos = rclcpp::QoS(5).reliable();
  pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
    "/localization/raw_position", qos);

  if (server_host_.empty() || server_port_ <= 0) {
    RCLCPP_INFO(get_logger(),
      "localization_client: server not configured "
      "(server_host empty or server_port<=0). Idling. "
      "EKF will run on IMU-only dead reckoning.");
  } else {
    RCLCPP_INFO(get_logger(),
      "localization_client: target %s:%d, publishing /localization/raw_position",
      server_host_.c_str(), server_port_);
  }

  io_thread_ = std::thread(&LocalizationClientNode::io_loop, this);
}

LocalizationClientNode::~LocalizationClientNode()
{
  running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
}

void LocalizationClientNode::handle_payload(const std::string & line)
{
  double x = 0.0, y = 0.0;
  if (!extract_xy(line, x, y)) {
    parse_errors_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  geometry_msgs::msg::PointStamped m;
  m.header.stamp    = now();
  m.header.frame_id = frame_id_;
  m.point.x = x;
  m.point.y = y;
  m.point.z = 0.0;
  pub_->publish(m);
  msg_count_.fetch_add(1, std::memory_order_relaxed);
}

void LocalizationClientNode::io_loop()
{
  // Idle mode: no endpoint configured. Sleep until shutdown — never connect,
  // never publish, never spam. The EKF will simply run on IMU-only and the
  // /localization/raw_position topic stays empty.
  while (running_ && (server_host_.empty() || server_port_ <= 0)) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  while (running_) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "socket() failed: %s", std::strerror(errno));
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_s_));
      continue;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(server_port_));

    if (::inet_pton(AF_INET, server_host_.c_str(), &addr.sin_addr) != 1) {
      addrinfo hints{};
      addrinfo * res = nullptr;
      hints.ai_family   = AF_INET;
      hints.ai_socktype = SOCK_STREAM;
      if (::getaddrinfo(server_host_.c_str(), nullptr, &hints, &res) != 0 || !res) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "DNS resolve failed for '%s'", server_host_.c_str());
        ::close(fd);
        std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_s_));
        continue;
      }
      addr.sin_addr = reinterpret_cast<sockaddr_in *>(res->ai_addr)->sin_addr;
      ::freeaddrinfo(res);
    }

    timeval tv{};
    tv.tv_sec  = static_cast<int>(recv_timeout_s_);
    tv.tv_usec = static_cast<int>((recv_timeout_s_ - tv.tv_sec) * 1e6);
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Connect %s:%d failed: %s",
        server_host_.c_str(), server_port_, std::strerror(errno));
      ::close(fd);
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_s_));
      continue;
    }

    RCLCPP_INFO(get_logger(), "Connected to localization server %s:%d",
      server_host_.c_str(), server_port_);

    std::string buf;
    buf.reserve(4096);
    char chunk[1024];

    while (running_) {
      ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
      if (n > 0) {
        buf.append(chunk, chunk + n);
        // Extract complete frames: prefer '\n' terminators; fall back to
        // '}' terminator for one-object-per-send servers that omit newline.
        while (true) {
          auto nl = buf.find('\n');
          auto rb = buf.find('}');
          size_t cut = std::string::npos;
          if (nl != std::string::npos && (rb == std::string::npos || nl < rb)) {
            cut = nl + 1;
          } else if (rb != std::string::npos) {
            cut = rb + 1;
          }
          if (cut == std::string::npos) break;
          std::string line = buf.substr(0, cut);
          buf.erase(0, cut);
          handle_payload(line);
        }
        // Cap the parser buffer to avoid unbounded growth on garbage input.
        if (buf.size() > 65536) buf.clear();
      } else if (n == 0) {
        RCLCPP_WARN(get_logger(), "Localization server closed connection");
        break;
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Localization recv timeout (server idle)");
          continue;
        }
        RCLCPP_WARN(get_logger(), "recv() error: %s", std::strerror(errno));
        break;
      }
    }

    ::close(fd);
    if (!running_) break;
    std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_s_));
  }
}

}  // namespace bfmc_fusion
