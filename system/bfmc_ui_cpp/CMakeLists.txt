cmake_minimum_required(VERSION 3.8)
project(bfmc_ui_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(bfmc_ui_node src/bfmc_ui_node.cpp)
ament_target_dependencies(bfmc_ui_node
  rclcpp std_msgs nav_msgs bfmc_msgs)

install(TARGETS bfmc_ui_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
