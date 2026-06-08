cmake_minimum_required(VERSION 3.8)
project(bfmc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneGeometry.msg"
  "msg/FreeSpace.msg"
  "msg/DetectedObject.msg"
  "msg/ObjectArray.msg"
  "msg/VehicleState.msg"
  "msg/Trajectory.msg"
  "msg/ControlCommand.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()

