cmake_minimum_required(VERSION 3.8)
project(object_tracker_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(object_tracker_node
  src/byte_tracker.cpp
  src/kalman_track.cpp
  src/object_tracker_node.cpp
  src/main.cpp
)

target_link_libraries(object_tracker_node Eigen3::Eigen)

ament_target_dependencies(object_tracker_node
  rclcpp
  std_msgs
  bfmc_msgs
)

install(TARGETS object_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
