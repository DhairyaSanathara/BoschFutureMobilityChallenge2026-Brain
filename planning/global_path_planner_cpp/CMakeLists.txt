cmake_minimum_required(VERSION 3.8)
project(global_path_planner_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
# libtinyxml2-dev on Ubuntu 22.04 ships the headers + .so but does not install
# tinyxml2Config.cmake. Locate it by hand so we don't depend on the upstream
# CMake config being present.
find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY  NAMES tinyxml2  REQUIRED)

# yaml-cpp — used by ZoneMap for the zones.yaml side-car (Phase Final).
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${TINYXML2_INCLUDE_DIR})

add_executable(global_path_planner_node
  src/graph.cpp
  src/graphml_parser.cpp
  src/planner.cpp
  src/zone_map.cpp
  src/planner_node.cpp
  src/main.cpp
)

ament_target_dependencies(global_path_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  bfmc_msgs
)

target_link_libraries(global_path_planner_node ${TINYXML2_LIBRARY} yaml-cpp)

install(TARGETS global_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
