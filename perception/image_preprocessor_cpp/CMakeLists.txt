cmake_minimum_required(VERSION 3.18)
project(image_preprocessor_cpp LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>
                      $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
                      $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "image_preprocessor_cpp: ARM NEON + Orin tuning")
  add_compile_options(-O3 -march=armv8-a+simd -ffast-math -funroll-loops)
endif()

# Jetson Orin Nano is SM 8.7 (Ampere). Same arch as object_detection_cpp.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(CUDAToolkit REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)

# ---------------------------------------------------------------------------
# Composable component library (C++ + CUDA)
# ---------------------------------------------------------------------------
add_library(image_preprocessor_component SHARED
  src/image_preprocessor_node.cpp
  src/preprocess_cuda.cu
)

set_target_properties(image_preprocessor_component PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(image_preprocessor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_preprocessor_component
  ${OpenCV_LIBS}
  CUDA::cudart
)

ament_target_dependencies(image_preprocessor_component ${DEPENDENCIES})

rclcpp_components_register_nodes(image_preprocessor_component
  "image_preprocessor_cpp::ImagePreprocessorNode"
)

# ---------------------------------------------------------------------------
# Standalone executable (fallback / debug)
# ---------------------------------------------------------------------------
add_executable(image_preprocessor_node src/main.cpp)
target_link_libraries(image_preprocessor_node image_preprocessor_component)
ament_target_dependencies(image_preprocessor_node ${DEPENDENCIES})

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS image_preprocessor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS image_preprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(image_preprocessor_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
