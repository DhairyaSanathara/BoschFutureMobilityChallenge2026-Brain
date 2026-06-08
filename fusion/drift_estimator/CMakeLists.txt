cmake_minimum_required(VERSION 3.8)
project(drift_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(drift_estimator_node
  src/drift_estimator_node.cpp
  src/main.cpp
)

ament_target_dependencies(drift_estimator_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  bfmc_msgs
)

install(TARGETS drift_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
