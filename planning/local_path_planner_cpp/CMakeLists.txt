cmake_minimum_required(VERSION 3.8)
project(local_path_planner_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

include_directories(include)

add_executable(local_path_planner_node
  src/local_planner.cpp
  src/local_planner_node.cpp
  src/main.cpp
)

ament_target_dependencies(local_path_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  bfmc_msgs
)

install(TARGETS local_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
