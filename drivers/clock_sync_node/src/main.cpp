#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <atomic>

#include "clock_sync_node/clock_sync_node.hpp"

static std::atomic<bool> running{true};

static void signal_handler(int)
{
  running.store(false);
}

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);

  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<clock_sync_node::ClockSyncNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && running.load())
  {
    executor.spin_some(std::chrono::milliseconds(1));
  }

  rclcpp::shutdown();
  return 0;
}
