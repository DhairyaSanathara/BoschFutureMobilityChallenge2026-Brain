cmake_minimum_required(VERSION 3.8)
project(v2x_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(v2x_bridge_node
  src/v2x_bridge_node.cpp
  src/main.cpp
)

ament_target_dependencies(v2x_bridge_node
  rclcpp
  std_msgs
  bfmc_msgs
)

install(TARGETS v2x_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
