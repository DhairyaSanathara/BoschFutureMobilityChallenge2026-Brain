cmake_minimum_required(VERSION 3.8)
project(safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

# Include headers
include_directories(include)

# =============================
# Safety Supervisor Node
# =============================
add_executable(safety_supervisor_node
  src/safety_supervisor_node.cpp
)

ament_target_dependencies(
  safety_supervisor_node
  rclcpp
  std_msgs
  sensor_msgs
  bfmc_msgs
)

# =============================
# Watchdog Node
# =============================
add_executable(watchdog_node
  src/watchdog_node.cpp
)

ament_target_dependencies(
  watchdog_node
  rclcpp
  std_msgs
  bfmc_msgs
)

# =============================
# Emergency Stop Aggregator (Phase 7)
# =============================
add_executable(emergency_stop_node
  src/emergency_stop_node.cpp
)
ament_target_dependencies(
  emergency_stop_node
  rclcpp std_msgs bfmc_msgs
)

# =============================
# System State Machine (Phase B)
# =============================
add_executable(system_state_machine_node
  src/system_state_machine_node.cpp
)
ament_target_dependencies(
  system_state_machine_node
  rclcpp std_msgs sensor_msgs nav_msgs bfmc_msgs
)

# =============================
# Install executables
# =============================
install(TARGETS
  safety_supervisor_node
  watchdog_node
  emergency_stop_node
  system_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
