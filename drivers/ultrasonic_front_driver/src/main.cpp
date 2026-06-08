#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include "ultrasonic_front_driver/ultrasonic_front_driver_node.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int /*signum*/)
{
  g_running.store(false);
}

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ultrasonic_front_driver::UltrasonicFrontNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_running.load()) {
    executor.spin_some(std::chrono::milliseconds(1));
  }

  rclcpp::shutdown();
  return 0;
}
