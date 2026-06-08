cmake_minimum_required(VERSION 3.8)
project(bfmc_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_controller_node
  src/main.cpp
  src/trajectory_controller_node.cpp
)

target_include_directories(trajectory_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_controller_node
  rclcpp
  rclcpp_lifecycle
  bfmc_msgs
)

target_link_libraries(trajectory_controller_node
  Eigen3::Eigen
)

# Race-day manual mode: Twist → ControlCommand bridge for teleop_twist_keyboard.
add_executable(teleop_to_control_node src/teleop_to_control_node.cpp)
ament_target_dependencies(teleop_to_control_node
  rclcpp geometry_msgs bfmc_msgs)

install(TARGETS
    trajectory_controller_node
    teleop_to_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
