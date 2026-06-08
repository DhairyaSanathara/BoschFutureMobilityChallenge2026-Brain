cmake_minimum_required(VERSION 3.8)
project(bfmc_ui_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

add_executable(bfmc_ui_server_node src/bfmc_ui_server_node.cpp)
ament_target_dependencies(bfmc_ui_server_node rclcpp)
target_link_libraries(bfmc_ui_server_node Threads::Threads)

install(TARGETS bfmc_ui_server_node DESTINATION lib/${PROJECT_NAME})

# Static assets: the Bosch React build (lives at bfmc_ui/Bfmc_userinterface/dist
# at the repo root) and our companion dashboard.html. The launch file points
# the server at this share dir via the `web_root` parameter, so the install
# layout is the source of truth at run-time.
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../bfmc_ui/Bfmc_userinterface/dist/
  DESTINATION share/${PROJECT_NAME}/web)

# Overlay our companion files on top of the React build. dashboard.html sits
# alongside index.html so the operator can navigate /dashboard.html.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/
  DESTINATION share/${PROJECT_NAME}/web
  FILES_MATCHING PATTERN "*.html" PATTERN "*.js" PATTERN "*.css")

ament_package()
