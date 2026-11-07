# BoschFutureMobilityChallenge2026-Brain

Jetson-side autonomy stack for the OdyssiX vehicle entered in the Bosch Future Mobility Challenge 2026.

This repository is the **Brain**: perception, planning, control, fusion, safety supervision, and operator UI running on an NVIDIA Jetson Orin Nano under ROS 2 Humble. It pairs with the companion **Embedded** repository (`BoschFutureMobilityChallenge2026-Embedded`) which runs on the on-vehicle STM32F401RE Nucleo and handles the BNO085 IMU read loop and PWM steering/ESC output.

The two repositories communicate over UART using micro-ROS / XRCE-DDS at 460 800 baud.

## Platform

| Layer | Choice |
|---|---|
| Compute | NVIDIA Jetson Orin Nano |
| OS | Ubuntu 22.04 / JetPack 6.x |
| ROS | ROS 2 Humble |
| RMW | `rmw_cyclonedds_cpp` |
| ROS_DOMAIN_ID | 42 |
| Container | Docker (drivers only — autonomy runs native) |
| Languages | C++ (autonomy stack), Python (launch + setup) |
| Build | colcon |

## Repository layout

```
.
├── bfmc_msgs/             ROS 2 message contracts (ControlCommand, VehicleState, …)
├── bfmc_ui/               Bosch reference UI (BSD-3-Clause, upstream). See AUTHORSHIP.md.
├── control/               Trajectory controller + teleop bridge (package: bfmc_control)
├── docker/                Drivers container: RealSense + micro-ROS agent
├── docs/                  Architecture, subsystems, design decisions, trade-offs, hardware
├── drivers/               imu_driver (deployed), clock_sync_node + ultrasonic_front_driver (built, not in deployed launch)
├── fusion/                ekf_localization, drift_estimator, tf2_publisher, vehicle_state
├── perception/            image_preprocessor_cpp, lane_geometry_cpp, free_space_cpp, object_detection_cpp, object_tracker_cpp
├── planning/              behavior_planner_cpp, global_path_planner_cpp, local_path_planner_cpp, mission_supervisor_cpp, trajectory_generator
├── safety/                watchdog, emergency_stop, supervisor, state machine
├── system/                bfmc_ui_cpp + bfmc_ui_server, health_monitor_cpp, runtime_telemetry_cpp, v2x_bridge_cpp
├── system_bringup/        Launch files (race_day, hil, lane_follow, manual_drive, manual_data_collection) + CycloneDDS profile
└── tools/                 Operator scripts
```

## Quick start

### Prerequisites

- Jetson Orin Nano with JetPack 6.x (Ubuntu 22.04).
- ROS 2 Humble installed system-wide.
- Docker + docker-compose for the drivers container.
- The companion vehicle hardware: STM32F401RE running `BoschFutureMobilityChallenge2026-Embedded`, RealSense D435i camera.

### Build

```bash
mkdir -p ~/bfmc_ws/src
ln -s "$(pwd)" ~/bfmc_ws/src/bfmc_autonomy
cd ~/bfmc_ws
colcon build --symlink-install
```

Apt-installed ROS packages used by the deployed launch:

```
ros-humble-nav2-lifecycle-manager
ros-humble-rclcpp-components
ros-humble-rosbridge-server
ros-humble-tf2-ros
ros-humble-tf2-web-republisher    # optional; launch skips silently if absent
ros-humble-realsense2-camera
ros-humble-rmw-cyclonedds-cpp
```

### Drivers container

```bash
cd docker
docker compose up --build
```

This brings up the `bfmc_drivers` container which runs `realsense2_camera` and the micro-ROS agent on `/dev/ttyACM0` at 460 800 baud.

### Autonomy stack (native)

In a separate terminal, with the workspace sourced:

```bash
source ~/bfmc_ws/install/setup.bash
ros2 launch system_bringup race_day.launch.py
```

### TensorRT engine path (operator-supplied)

`object_detection_node` requires a TensorRT engine file that the operator builds on-device from an ONNX model. The path is **not** baked into the repository. Supply it one of two ways:

1. Edit `perception/object_detection_cpp/config/object_detection.yaml` and set `model.engine_path:` to your absolute path; or
2. Pass `-p engine_path:=/abs/path/to/yolov8n_fp16.engine` to `object_detection_node` (e.g., from your launch file).

If `engine_path` is left empty, `object_detection_node` throws a clear startup error.

Other launch entry points are available:

| Launch file | Purpose |
|---|---|
| `race_day.launch.py` | Deployed stack. Reference launch. |
| `hil_bringup.launch.py` | Hardware-in-the-loop testing. |
| `lane_follow.launch.py` | Lane-follow demo without mission supervisor. |
| `manual_drive.launch.py` | Teleop. |
| `manual_data_collection.launch.py` | Drivers + perception only. |

### Operator UI

With the deployed launch running, open in a browser:

- `http://<jetson-ip>:8080/` — the legacy Bosch React UI (served from `bfmc_ui/Bfmc_userinterface/dist/`).
- `http://<jetson-ip>:8080/dashboard.html` — the OdyssiX companion dashboard.
- WebSocket at `ws://<jetson-ip>:9090` is the rosbridge feed.

## Documentation

| Document | Purpose |
|---|---|
| `docs/architecture/system_overview.md` | End-to-end dataflow, topic graph, deployment graph. |
| `docs/subsystems/ros2/packages.md` | Per-package roster + publishers / subscribers / parameters. |
| `docs/subsystems/ros2/node_lifecycle_and_qos.md` | Lifecycle nodes, composable container, executors, QoS. |
| `docs/subsystems/docker/docker_audit.md` | Drivers container detail. |
| `docs/subsystems/micro_ros/micro_ros_integration.md` | UART micro-ROS path and failure modes. |
| `docs/subsystems/vehicle_pipeline/pipeline.md` | Deployed stage-by-stage pipeline. |
| `docs/hardware/hardware_overview.md` | Compute, sensing, actuation, power, networking. |
| `docs/design_decisions/decisions.md` | Recorded architectural choices. |
| `docs/tradeoffs/tradeoffs.md` | Trade-off matrices for the major choices. |

## Companion repository

The vehicle firmware lives in `BoschFutureMobilityChallenge2026-Embedded`. Topics across the bridge:

- STM32 → Brain: `/microros/imu_raw` (`sensor_msgs/Imu`, consumed by `imu_driver_node`). This is the firmware's only publisher.
- Brain → STM32: `/control/command` (`bfmc_msgs/ControlCommand`).

The firmware applies a 500 ms command-silence deadman: if `/control/command` stops arriving, the STM32 returns the steering servo and ESC to neutral PWM within 500 ms.

## Hardware

See `docs/hardware/hardware_overview.md` for the topology, USB layout, TF tree, and component list. Mechanical drawings and wiring harnesses are out of scope for this software repository.

## License

This project is licensed under the Apache License, Version 2.0. See `LICENSE`.

Third-party code retains its original licensing:

- `bfmc_ui/` — Bosch / BFMC organizers reference UI (BSD-3-Clause). See `bfmc_ui/LICENSE`.
- Apt-installed ROS 2 packages — their respective upstream licenses.

See `AUTHORSHIP.md` for a third-party attribution inventory.

## Status

The repository represents the OdyssiX BFMC 2026 race-day deployment of the Brain. The deployed launch is `system_bringup/launch/race_day.launch.py`. Three Python prototype packages (image_preprocessor, behavior_planner, local_path_planner) were superseded by their C++ counterparts and are not part of this public release.

## Reporting issues

See `SECURITY.md` for the safety boundary and reporting guidance. Standard issues / PRs are welcome via the project's issue tracker.
