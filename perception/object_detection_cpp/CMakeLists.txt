cmake_minimum_required(VERSION 3.18)
project(object_detection_cpp LANGUAGES CXX CUDA)

# ------------------------------------------------------------------------------
# Compiler & build type
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wpedantic on the host compiler flags every nvcc-generated line directive,
  # so apply it only to plain C++ sources (not the CUDA host pass).
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>
                      $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
                      $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-O3 -march=armv8-a+simd -ffast-math -funroll-loops)
endif()

# Jetson Orin Nano is SM 8.7 (Ampere). Build only that arch to keep PTX small.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

# yaml-cpp (no ROS find_package shim on Humble for some installs — fall through)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h)
  find_library(YAML_CPP_LIBRARY yaml-cpp)
endif()

# TensorRT (header + libnvinfer). Jetson packages live in /usr.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include
)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib
)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT not found. Install libnvinfer-dev.")
endif()
message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT library: ${TENSORRT_LIBRARY}")

set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  sensor_msgs
  bfmc_msgs
)

# ------------------------------------------------------------------------------
# Library: detection core (C++ + CUDA)
# ------------------------------------------------------------------------------
add_library(object_detection_core SHARED
  src/object_detection_node.cpp
  src/trt_engine.cpp
  src/postprocess.cpp
  src/health_monitor.cpp
  src/config_loader.cpp
  src/preprocess_cuda.cu
)

set_target_properties(object_detection_core PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(object_detection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR}
)

if(yaml-cpp_FOUND)
  target_link_libraries(object_detection_core
    yaml-cpp
  )
else()
  target_include_directories(object_detection_core PUBLIC ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(object_detection_core ${YAML_CPP_LIBRARY})
endif()

target_link_libraries(object_detection_core
  ${TENSORRT_LIBRARY}
  ${TENSORRT_PLUGIN_LIBRARY}
  CUDA::cudart
)

ament_target_dependencies(object_detection_core ${DEPENDENCIES})

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(object_detection_node src/main.cpp)
target_link_libraries(object_detection_node object_detection_core)
ament_target_dependencies(object_detection_node ${DEPENDENCIES})

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS object_detection_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS object_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/benchmark_latency.sh
  scripts/probe_engine.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(object_detection_core)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
