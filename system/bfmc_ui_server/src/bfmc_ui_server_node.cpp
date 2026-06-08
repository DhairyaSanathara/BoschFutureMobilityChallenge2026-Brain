// Race-day BFMC UI static-file HTTP server.
//
// Serves Bfmc_userinterface/dist/ (the legacy Bosch React build) + our
// dashboard.html companion page from a single HTTP port. The browser
// connects to ws://<host>:9090 (rosbridge) directly — this process is just
// the file server. POSIX sockets, no external deps.
//
// Endpoints:
//   GET /                  → index.html  (React UI)
//   GET /dashboard.html    → companion dashboard (consumes /ui/status)
//   GET /preview.html      → React preview page
//   GET /assets/*          → JS / CSS / images
//   GET /healthz           → "ok" (probe)
//   anything else          → 404
//
// One thread per connection. Path traversal blocked. Concurrent requests
// don't block the main rclcpp spin (the listener thread is detached and
// the per-connection handlers join independently).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{

// MIME map. Keep tight — only what the React build emits + a few extras.
const std::unordered_map<std::string, std::string> kMime = {
  {".html", "text/html; charset=utf-8"},
  {".htm",  "text/html; charset=utf-8"},
  {".js",   "application/javascript; charset=utf-8"},
  {".mjs",  "application/javascript; charset=utf-8"},
  {".css",  "text/css; charset=utf-8"},
  {".svg",  "image/svg+xml"},
  {".png",  "image/png"},
  {".jpg",  "image/jpeg"},
  {".jpeg", "image/jpeg"},
  {".gif",  "image/gif"},
  {".ico",  "image/x-icon"},
  {".json", "application/json; charset=utf-8"},
  {".woff", "font/woff"},
  {".woff2","font/woff2"},
  {".ttf",  "font/ttf"},
  {".map",  "application/json; charset=utf-8"},
};

std::string mime_for(const std::string & path)
{
  const auto pos = path.find_last_of('.');
  if (pos == std::string::npos) return "application/octet-stream";
  const auto ext = path.substr(pos);
  const auto it = kMime.find(ext);
  return it == kMime.end() ? "application/octet-stream" : it->second;
}

// Send everything on the socket; returns false on first short write.
bool write_all(int fd, const char * data, std::size_t n)
{
  while (n > 0) {
    const ssize_t w = ::send(fd, data, n, MSG_NOSIGNAL);
    if (w <= 0) return false;
    data += w;
    n    -= static_cast<std::size_t>(w);
  }
  return true;
}

// Read a single HTTP request line + headers, up to 4 KiB. We don't support
// request bodies (GET-only). Returns the start-line (e.g. "GET / HTTP/1.1")
// or empty on error.
std::string read_request_(int fd)
{
  std::string buf;
  buf.reserve(1024);
  char chunk[1024];
  for (;;) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0) break;
    buf.append(chunk, static_cast<std::size_t>(n));
    if (buf.size() > 4096) break;
    if (buf.find("\r\n\r\n") != std::string::npos) break;
  }
  const auto nl = buf.find("\r\n");
  return nl == std::string::npos ? std::string{} : buf.substr(0, nl);
}

// Parse "GET <path> HTTP/1.x" → path (URL-decoded). Returns empty on parse
// error or non-GET. Also strips query string.
std::string parse_path_(const std::string & start_line)
{
  if (start_line.compare(0, 4, "GET ") != 0) return {};
  const auto sp2 = start_line.find(' ', 4);
  if (sp2 == std::string::npos) return {};
  std::string p = start_line.substr(4, sp2 - 4);
  // strip query string
  const auto q = p.find('?');
  if (q != std::string::npos) p.resize(q);
  // crude URL decode
  std::string out;
  out.reserve(p.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    if (p[i] == '%' && i + 2 < p.size()) {
      const auto hex = p.substr(i + 1, 2);
      char * end = nullptr;
      const long v = std::strtol(hex.c_str(), &end, 16);
      if (end == hex.c_str() + 2) {
        out.push_back(static_cast<char>(v));
        i += 2;
        continue;
      }
    }
    out.push_back(p[i]);
  }
  return out;
}

// Resolve <web_root>/<path> with path-traversal guard. Returns empty
// string on rejection. We forbid any segment ".." or absolute paths.
std::string resolve_file_(const std::string & web_root, const std::string & url_path)
{
  if (url_path.empty() || url_path[0] != '/') return {};
  std::string p = url_path;
  if (p == "/") p = "/index.html";
  // Reject ..
  if (p.find("..") != std::string::npos) return {};
  return web_root + p;
}

bool file_exists_(const std::string & path)
{
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string slurp_(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void send_response_(int fd, int status, const std::string & status_text,
                    const std::string & mime, const std::string & body)
{
  std::ostringstream hdr;
  hdr << "HTTP/1.1 " << status << " " << status_text << "\r\n"
      << "Content-Type: "   << mime << "\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      << "Cache-Control: no-cache\r\n"
      // CORS so the operator's laptop on the same LAN can hit the page
      // without hassle. We only serve GET so this is harmless.
      << "Access-Control-Allow-Origin: *\r\n"
      << "Connection: close\r\n"
      << "\r\n";
  const auto h = hdr.str();
  write_all(fd, h.data(), h.size());
  if (!body.empty()) write_all(fd, body.data(), body.size());
}

void handle_connection_(int fd, const std::string & web_root,
                        const rclcpp::Logger & logger)
{
  const auto line = read_request_(fd);
  const auto path = parse_path_(line);

  if (path == "/healthz") {
    send_response_(fd, 200, "OK", "text/plain", "ok\n");
    ::close(fd);
    return;
  }

  const auto fs_path = resolve_file_(web_root, path.empty() ? "/" : path);
  if (fs_path.empty() || !file_exists_(fs_path)) {
    RCLCPP_DEBUG(logger, "404 %s (resolved=%s)", path.c_str(), fs_path.c_str());
    send_response_(fd, 404, "Not Found", "text/plain", "404\n");
    ::close(fd);
    return;
  }

  const auto body = slurp_(fs_path);
  send_response_(fd, 200, "OK", mime_for(fs_path), body);
  ::close(fd);
}

}  // namespace

class BfmcUiServerNode : public rclcpp::Node
{
public:
  BfmcUiServerNode()
  : rclcpp::Node("bfmc_ui_server_node")
  {
    // Race-day FixB: register a shutdown callback so the listen fd is
    // closed BEFORE rclcpp::spin() returns. This unblocks accept() in
    // the listener thread immediately on Ctrl-C / SIGTERM. Without this
    // the destructor wouldn't run until shared_ptr release, and we'd
    // hang for 15 s waiting for launch's SIGTERM/SIGKILL escalation.
    rclcpp::on_shutdown([this]() { this->stop_listener_(); });

    port_     = declare_parameter<int>        ("port",     8080);
    web_root_ = declare_parameter<std::string>("web_root", "");

    if (web_root_.empty()) {
      // Default: <install>/share/bfmc_ui_server/web
      const char * env = std::getenv("AMENT_PREFIX_PATH");
      if (env) {
        std::string paths{env};
        std::stringstream ss(paths);
        std::string p;
        while (std::getline(ss, p, ':')) {
          const auto candidate = p + "/share/bfmc_ui_server/web";
          struct stat st{};
          if (::stat(candidate.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            web_root_ = candidate;
            break;
          }
        }
      }
    }

    if (web_root_.empty()) {
      RCLCPP_ERROR(get_logger(),
        "bfmc_ui_server: web_root not set and could not auto-detect. "
        "Set the `web_root` parameter to the dist/ directory.");
      return;
    }

    start_listener_();
  }

  ~BfmcUiServerNode() override
  {
    stop_listener_();
  }

private:
  void stop_listener_()
  {
    // Idempotent: safe to call from on_shutdown() AND from the destructor.
    if (stop_.exchange(true)) return;
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (listener_.joinable()) listener_.join();
  }

public:

private:
  void start_listener_()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "bfmc_ui_server: socket() failed: %s",
                   std::strerror(errno));
      return;
    }
    int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(),
        "bfmc_ui_server: bind(0.0.0.0:%d) failed: %s",
        port_, std::strerror(errno));
      ::close(listen_fd_); listen_fd_ = -1;
      return;
    }
    if (::listen(listen_fd_, 16) < 0) {
      RCLCPP_ERROR(get_logger(),
        "bfmc_ui_server: listen() failed: %s", std::strerror(errno));
      ::close(listen_fd_); listen_fd_ = -1;
      return;
    }

    RCLCPP_INFO(get_logger(),
      "bfmc_ui_server: serving %s on 0.0.0.0:%d  (open http://<host>:%d/ or /dashboard.html)",
      web_root_.c_str(), port_, port_);

    listener_ = std::thread([this]() {
      while (!stop_.load()) {
        sockaddr_in peer{};
        socklen_t   plen = sizeof(peer);
        const int   fd   = ::accept(listen_fd_,
                                    reinterpret_cast<sockaddr *>(&peer), &plen);
        if (fd < 0) {
          if (stop_.load()) break;
          continue;
        }
        // One detached thread per connection. Short-lived (slurp + send).
        std::thread([fd, root = web_root_, logger = get_logger()]() {
          handle_connection_(fd, root, logger);
        }).detach();
      }
    });
  }

  int          port_{8080};
  std::string  web_root_;
  int          listen_fd_{-1};
  std::thread  listener_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BfmcUiServerNode>());
  rclcpp::shutdown();
  return 0;
}
