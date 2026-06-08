cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

# C++17 required
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ---------- dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# ---------- executable ----------
add_executable(trajectory_generator_node
  src/main.cpp
  src/trajectory_generator_node.cpp
)

target_include_directories(trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_generator_node
  rclcpp
  std_msgs
  bfmc_msgs
  geometry_msgs
)

target_link_libraries(trajectory_generator_node
  nlohmann_json::nlohmann_json
)

# ---------- install ----------
install(TARGETS trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------- tests ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
