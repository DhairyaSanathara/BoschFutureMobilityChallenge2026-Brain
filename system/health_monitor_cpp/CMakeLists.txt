cmake_minimum_required(VERSION 3.8)
project(health_monitor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(health_monitor_node
  src/health_monitor_node.cpp
  src/main.cpp
)

ament_target_dependencies(health_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2_msgs
  bfmc_msgs
)

install(TARGETS health_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
