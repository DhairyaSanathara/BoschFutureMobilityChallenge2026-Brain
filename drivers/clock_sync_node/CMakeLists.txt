cmake_minimum_required(VERSION 3.8)
project(clock_sync_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(clock_sync_node_exec
  src/clock_sync_node.cpp
  src/main.cpp
)

target_include_directories(clock_sync_node_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(clock_sync_node_exec
  rclcpp
  builtin_interfaces
)

install(TARGETS clock_sync_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
