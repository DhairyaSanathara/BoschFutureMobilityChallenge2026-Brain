cmake_minimum_required(VERSION 3.8)
project(lane_geometry_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-O3 -march=armv8-a+simd -ffast-math -funroll-loops)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  bfmc_msgs
  cv_bridge
)

# ---------------------------------------------------------------------------
# Composable component library
# ---------------------------------------------------------------------------
add_library(lane_geometry_component SHARED
  src/lane_geometry_node.cpp
)

target_include_directories(lane_geometry_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lane_geometry_component
  ${OpenCV_LIBS}
)

ament_target_dependencies(lane_geometry_component ${DEPENDENCIES})

rclcpp_components_register_nodes(lane_geometry_component
  "lane_geometry_cpp::LaneGeometryNode"
)

# ---------------------------------------------------------------------------
# Standalone executable
# ---------------------------------------------------------------------------
add_executable(lane_geometry_node src/main.cpp)
target_link_libraries(lane_geometry_node lane_geometry_component)
ament_target_dependencies(lane_geometry_node ${DEPENDENCIES})

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS lane_geometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lane_geometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(lane_geometry_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
