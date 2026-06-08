cmake_minimum_required(VERSION 3.8)
project(mission_supervisor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bfmc_msgs REQUIRED)

find_package(tinyxml2_vendor REQUIRED)
find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY  NAMES tinyxml2  REQUIRED)

find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${TINYXML2_INCLUDE_DIR})

add_executable(mission_semantics_node
  src/mission_graph.cpp
  src/mission_semantics_node.cpp
)

ament_target_dependencies(mission_semantics_node
  rclcpp
  ament_index_cpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  bfmc_msgs
)

target_link_libraries(mission_semantics_node ${TINYXML2_LIBRARY} yaml-cpp)

# Audit tool — standalone (no ROS dep) BFS gap-filler that produces
# mission_expanded.yaml. Run by hand:
#   ros2 run mission_supervisor_cpp mission_expander \
#     <graphml> <mission_in.yaml> <mission_out.yaml>
add_executable(mission_expander
  src/mission_graph.cpp
  src/mission_expander_main.cpp
)
target_link_libraries(mission_expander ${TINYXML2_LIBRARY} yaml-cpp)

add_executable(mission_semantics_inherit
  src/mission_graph.cpp
  src/mission_semantics_inherit_main.cpp
)
target_link_libraries(mission_semantics_inherit ${TINYXML2_LIBRARY} yaml-cpp)

add_executable(mission_trace
  src/mission_graph.cpp
  src/mission_trace_main.cpp
)
target_link_libraries(mission_trace ${TINYXML2_LIBRARY} yaml-cpp)

install(TARGETS
    mission_semantics_node
    mission_expander
    mission_semantics_inherit
    mission_trace
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# Install the canonical configs into the package share so the launch file
# can resolve them from the install tree (no source-tree dependency at run-time).
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "competition_*.yaml"
)

ament_package()
