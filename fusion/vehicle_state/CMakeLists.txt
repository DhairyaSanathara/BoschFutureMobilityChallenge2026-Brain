cmake_minimum_required(VERSION 3.8)
project(vehicle_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(vehicle_state_node
  src/vehicle_state_node.cpp
  src/main.cpp
)

ament_target_dependencies(vehicle_state_node
  rclcpp
  rclcpp_lifecycle
  bfmc_msgs
)

install(TARGETS vehicle_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
