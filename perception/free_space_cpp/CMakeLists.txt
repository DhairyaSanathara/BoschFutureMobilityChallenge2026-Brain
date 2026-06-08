cmake_minimum_required(VERSION 3.8)
project(free_space_cpp)

# ------------------------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Build type (default Release)
# ------------------------------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------------------------
# Jetson / ARM Performance Optimizations
# ------------------------------------------------------------------------------

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")

  message(STATUS "Building with ARM NEON optimizations")

  add_compile_options(
    -O3
    -march=armv8-a+simd
    -ffast-math
    -funroll-loops
  )

endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  bfmc_msgs
)

# ------------------------------------------------------------------------------
# Library
# ------------------------------------------------------------------------------

add_library(free_space_lib SHARED
  src/free_space_node.cpp
)

target_include_directories(free_space_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(free_space_lib ${DEPENDENCIES})

rclcpp_components_register_nodes(free_space_lib
  "free_space_cpp::FreeSpaceNode"
)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------

add_executable(free_space_node
  src/main.cpp
)

target_link_libraries(free_space_node
  free_space_lib
)

ament_target_dependencies(free_space_node ${DEPENDENCIES})

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS free_space_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS free_space_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Ament packaging
# ------------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(free_space_lib)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
