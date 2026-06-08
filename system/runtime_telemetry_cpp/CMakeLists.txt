cmake_minimum_required(VERSION 3.8)
project(runtime_telemetry_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(runtime_telemetry_node src/runtime_telemetry_node.cpp)
ament_target_dependencies(runtime_telemetry_node rclcpp std_msgs)

install(TARGETS runtime_telemetry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
