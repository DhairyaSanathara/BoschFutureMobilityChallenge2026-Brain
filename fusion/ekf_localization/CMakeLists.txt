cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ---- EKF lifecycle node ------------------------------------------------------
add_executable(ekf_localization_node
  src/ekf_localization_node.cpp
  src/main.cpp
)

target_link_libraries(ekf_localization_node
  Eigen3::Eigen
)

ament_target_dependencies(ekf_localization_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  geometry_msgs
  bfmc_msgs
  tf2
  tf2_geometry_msgs
)

# ---- Localization client (TCP-JSON ingest) -----------------------------------
add_executable(localization_client_node
  src/localization_client_node.cpp
  src/localization_client_main.cpp
)

ament_target_dependencies(localization_client_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  ekf_localization_node
  localization_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
