cmake_minimum_required(VERSION 3.8)
project(tf2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(tf2_publisher_node
  src/tf2_publisher_node.cpp
  src/main.cpp
)

ament_target_dependencies(tf2_publisher_node
  rclcpp
  rclcpp_lifecycle
  bfmc_msgs
  geometry_msgs
  tf2_ros
  tf2
)

install(TARGETS tf2_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
