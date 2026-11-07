# System Overview — OdyssiX BFMC 2026 Brain

Generated 2026-06-08. Grounded in the current state of this repository.

The Brain is the Jetson Orin Nano side of the OdyssiX BFMC 2026 vehicle. It runs the perception → planning → control loop, manages safety supervision, and serves the operator UI. It talks to the STM32 vehicle firmware over micro-ROS / XRCE-DDS on UART, and to the RealSense D435i camera over USB.

## High-level dataflow

```
Operator UI (browser)
        │
        ▼
   ┌────────────────────────────────────────────────────┐
   │ Jetson Orin Nano (Ubuntu 22.04 + JetPack 6.x)      │
   │                                                    │
   │  Docker: bfmc_drivers ──────────────────────────┐  │
   │  │                                             │  │
   │  │  realsense2_camera (RealSense D435i)        │  │
   │  │  micro_ros_agent (UART, 460800 baud)        │  │
   │  └─────────────────────────────────────────────┘  │
   │                       │                            │
   │                       ▼ (ROS 2 Humble, CycloneDDS) │
   │  Native: race_day.launch.py — autonomy stack       │
   │    Perception → Planning → Control → Safety        │
   │    Fusion ⇄ TF ⇄ Lifecycle Manager (nav2)          │
   │    UI: bfmc_ui_node + bfmc_ui_server + rosbridge   │
   └────────────────────────────────────────────────────┘
        │                              ▲
        ▼                              │
   /control/command (ROS 2 topic)      /microros/imu_raw
        │                              │
        ▼                              │
   micro_ros_agent (XRCE-DDS over UART)
        │                              ▲
        ▼ (460800 baud)                │
   STM32F401RE (vehicle firmware)
   PWM steering + ESC                  BNO085 IMU
```

## Platform

| Layer | Choice |
|---|---|
| Compute | NVIDIA Jetson Orin Nano |
| OS | Ubuntu 22.04 / JetPack 6.x |
| ROS | ROS 2 Humble |
| RMW | `rmw_cyclonedds_cpp` (CycloneDDS) |
| ROS_DOMAIN_ID | `42` (set in launch + Dockerfile) |
| Containerisation | Docker (drivers container only; autonomy runs native) |
| Languages | C++ (perception, planning, control, fusion, safety), Python (launch + setup) |
| Build | colcon (ament_cmake + ament_python) |

## Deployed packages and entry points

The deployed (race-day) launch is `system_bringup/launch/race_day.launch.py`. It composes the following packages.

### Drivers (host side)

| Package | Executable | Role |
|---|---|---|
| `imu_driver` | `imu_driver_node` | Consumes `/microros/imu_raw` and republishes it on `/imu/data_filtered`. |
| `ekf_localization` | `localization_client_node` | TCP client for an external localization server. Configured with `server_host=''`, `server_port=0` → effectively disabled in race_day. |
| `runtime_telemetry_cpp` | `runtime_telemetry_node` | Reads `/proc/stat`, `/proc/meminfo`, `/sys/class/thermal/*/temp` and publishes `/system/runtime_health`. |

### Perception (ROS 2 composable container + standalone nodes)

A `rclcpp_components::component_container_mt` named `perception_container` hosts three composable lifecycle nodes that share intra-process communication:

| Plugin | Package | Lifecycle? | Role |
|---|---|---|---|
| `image_preprocessor_cpp::ImagePreprocessorNode` | `image_preprocessor_cpp` | yes | Subscribes `/camera/camera/color/image_raw`; CUDA preprocess; publishes `/perception/image_preprocessed`. |
| `lane_geometry_cpp::LaneGeometryNode` | `lane_geometry_cpp` | yes | Lane geometry extraction. Publishes `/perception/lane_geometry`. |
| `free_space_cpp::FreeSpaceNode` | `free_space_cpp` | yes | Free-space mask. Publishes `/perception/free_space` (rate-limited via `min_period_s=0.10`). |

Two perception nodes run as standalone (not in the composable container):

| Package | Executable | Role |
|---|---|---|
| `object_detection_cpp` | `object_detection_node` | TensorRT YOLO. Subscribes `/camera/camera/color/image_raw`; publishes `/perception/objects` (rate-limited via `min_period_s=0.125`, i.e. 8 Hz). |
| `object_tracker_cpp` | `object_tracker_node` | ByteTrack + Kalman. Subscribes `/perception/objects`; publishes `/perception/tracked_objects`. |

### Planning

| Package | Executable | Lifecycle? | Role |
|---|---|---|---|
| `global_path_planner_cpp` | `global_path_planner_node` | no | Loads `Competition_track_graph.graphml` + `zones.yaml`. Race-day runs with `disable_pub=True` so it stays in memory but defers to the mission supervisor for `/planning/global_path`. |
| `mission_supervisor_cpp` | `mission_semantics_node` | no (10 Hz) | Owns `/planning/global_path` and `/navigation/flags`. Consumes the BFS-expanded mission + inheritance-tagged semantics YAMLs. |
| `local_path_planner_cpp` | `local_path_planner_node` | no (20 Hz) | Reads `/planning/global_path` and `/perception/lane_geometry`; publishes `/planning/local_path` + `/planning/path_confidence`. |
| `trajectory_generator` | `trajectory_generator_node` | no | Reads `/planning/local_path`; publishes `/planning/trajectory`. |
| `behavior_planner_cpp` | `behavior_planner_node` | yes (5 Hz) | FSM. Reads `/planning/local_path`, `/perception/tracked_objects`, `/planning/current_region`, `/planning/current_speed_limit`. Publishes `/planning/behavior_state`, `/planning/target_speed`, `/planning/lateral_offset`, `/planning/competition_path`, `/planning/competition_markers`, `/planning/mission_progress`, `/planning/mission_score_estimate`. |
| `v2x_bridge_cpp` | `v2x_bridge_node` | no | UDP port 5007 + optional TCP. Publishes `/v2x/cars`, `/v2x/semaphores`. Race-day uses `enable_tcp=False`. |

### Control

| Package | Executable | Lifecycle? | Role |
|---|---|---|---|
| `bfmc_control` | `trajectory_controller_node` | yes (20 Hz) | Reads `/planning/trajectory`, `/planning/behavior_state`, `/planning/target_speed`. Publishes `/control/command_raw` (pre-supervisor) and consumes `/control/command` after the safety supervisor clamps it. |
| `bfmc_control` | `teleop_to_control_node` | no | Used by `manual_drive.launch.py` only — bridges `/cmd_vel` to `/control/command`. |

The control package builds two executables. Race-day uses only `trajectory_controller_node`.

### Fusion (lifecycle)

| Package | Executable | Role |
|---|---|---|
| `ekf_localization` | `ekf_localization_node` | EKF integrating IMU + (optionally) external localization. Publishes `/vehicle/state_raw`. |
| `drift_estimator` | `drift_estimator_node` | Lane-based yaw drift correction. Reads `/perception/lane_geometry`; publishes `/fusion/yaw_bias_correction` + debug topics. |
| `vehicle_state` | `vehicle_state_node` | Merges raw state + corrections. Publishes `/vehicle/state` and `/fusion/imu_confidence`. |
| `tf2_publisher` | `tf2_publisher_node` | Publishes `odom→base_link` TF from `/vehicle/state`. |

### Safety

`safety` ships four executables in one package.

| Executable | Role |
|---|---|
| `safety_supervisor_node` | Clamps `/control/command_raw` against `forward_max=0.35`, `reverse_max=0.0`, `deadband=0.05`, `max_brake=0.5`; publishes `/control/command`. |
| `emergency_stop_node` | Aggregates emergency signals. `emergency_persist_ticks=5` (1 s at 5 Hz) for soft signals; hard signals (manual / watchdog / vehicle_ready=false) are instant. Publishes `/safety/emergency`, `/safety/emergency_reason`, `/safety/emergency_stop`. |
| `system_state_machine_node` | 0.5 s tick. READY enter=20 valid samples (10 s), exit=50 invalid (25 s). Publishes `/system/system_state`, `/system/vehicle_ready`. |
| `watchdog_node` | Deferred 10 s after launch start. Monitors `/planning/trajectory` and emits `/safety/watchdog_alarm` on stall. |

### UI

| Package | Executable | Role |
|---|---|---|
| `bfmc_ui_cpp` | `bfmc_ui_node` | Aggregates status JSON @ 5 Hz, publishes `/ui/status`. |
| `bfmc_ui_server` | `bfmc_ui_server_node` | HTTP server on port 8080. Serves `bfmc_ui/Bfmc_userinterface/dist/` (upstream Bosch React build) + `dashboard.html`. |
| `rosbridge_server` (apt) | `rosbridge_websocket` | WebSocket on port 9090 — used by the Bosch React UI to read ROS topics. |
| `tf2_web_republisher` (apt, optional) | `tf2_web_republisher` | Streams TF over rosbridge for the map view. Launch silently skips if not installed. |

### Lifecycle orchestration

`nav2_lifecycle_manager` (apt) named `lifecycle_manager_autonomy` activates the eight lifecycle nodes 2 s after `trajectory_controller_node` spawns:

```
lane_geometry_node, free_space_node,
ekf_localization_node, drift_estimator_node,
vehicle_state_node, tf2_publisher_node,
behavior_planner_node, trajectory_controller_node
```

Settings: `autostart=True`, `bond_timeout=0.0`, `attempt_respawn_reconnection=False`.

## Topic graph (deployed race-day subset)

Topics are extracted from `create_publisher<...>` / `create_subscription<...>` literals across the C++ source.

### Hardware → drivers (UART via micro-ROS)

```
STM32 ──UART 460800──▶ micro_ros_agent ──/microros/imu_raw──▶ imu_driver_node
imu_driver_node ──/imu/data_filtered──▶ ekf_localization_node
```

### Camera

```
realsense2_camera ─/camera/camera/color/image_raw──▶ image_preprocessor_node, object_detection_node
                  ─/camera/camera/depth/image_rect_raw──▶ (subscriber: free_space_node)
                  ─/camera/camera/color/camera_info──▶ (calibration consumers)
```

### Perception chain

```
image_preprocessor_node ──/perception/image_preprocessed──▶ lane_geometry_node, free_space_node
lane_geometry_node      ──/perception/lane_geometry──────▶ local_path_planner_node, drift_estimator_node, behavior_planner_node
free_space_node         ──/perception/free_space──────────▶ local_path_planner_node
object_detection_node   ──/perception/objects────────────▶ object_tracker_node
object_tracker_node     ──/perception/tracked_objects────▶ behavior_planner_node
```

### Planning chain

```
mission_semantics_node ──/planning/global_path──────────▶ local_path_planner_node, behavior_planner_node, bfmc_ui_node
                       ──/navigation/flags────────────▶ behavior_planner_node
                       ──/planning/current_node──────▶ bfmc_ui_node
                       ──/planning/current_region────▶ behavior_planner_node
                       ──/planning/current_speed_limit─▶ behavior_planner_node, bfmc_ui_node
                       ──/planning/goal_node──────────▶ global_path_planner_node (informational)
local_path_planner_node ──/planning/local_path──────────▶ trajectory_generator_node, behavior_planner_node
                        ──/planning/path_confidence────▶ behavior_planner_node, bfmc_ui_node
trajectory_generator_node ──/planning/trajectory───────▶ trajectory_controller_node, watchdog_node
behavior_planner_node ──/planning/behavior_state───────▶ trajectory_controller_node, bfmc_ui_node
                      ──/planning/target_speed────────▶ trajectory_controller_node
                      ──/planning/lateral_offset──────▶ trajectory_controller_node
                      ──/planning/current_behavior────▶ bfmc_ui_node
                      ──/planning/mission_progress────▶ bfmc_ui_node
                      ──/planning/mission_score_estimate─▶ bfmc_ui_node
                      ──/planning/competition_path───▶ bfmc_ui_node
                      ──/planning/competition_markers─▶ bfmc_ui_node
v2x_bridge_node ──/v2x/cars────────▶ behavior_planner_node
                ──/v2x/semaphores──▶ behavior_planner_node
```

### Control / safety

```
trajectory_controller_node ──/control/command_raw──▶ safety_supervisor_node
                           ──/control/debug/clamped_throttle──▶ (debug)
                           ──/control/debug/raw_throttle──────▶ (debug)
                           ──/control/debug/supervisor_state──▶ (debug)
safety_supervisor_node    ──/control/command─────────▶ micro_ros_agent → STM32
emergency_stop_node       ──/safety/emergency──────▶ system_state_machine_node, trajectory_controller_node
                          ──/safety/emergency_reason
                          ──/safety/emergency_stop
watchdog_node             ──/safety/watchdog_alarm─▶ emergency_stop_node
system_state_machine_node ──/system/system_state──▶ trajectory_controller_node, bfmc_ui_node
                          ──/system/vehicle_ready─▶ trajectory_controller_node, bfmc_ui_node
```

### Fusion / state

```
ekf_localization_node ──/vehicle/state_raw──▶ vehicle_state_node
drift_estimator_node  ──/fusion/yaw_bias_correction──▶ vehicle_state_node
                      ──/fusion/debug/correction_active
                      ──/fusion/debug/curvature_weight
                      ──/fusion/debug/lane_heading
                      ──/fusion/debug/message_age_ms
                      ──/fusion/debug/node_latency_ms
                      ──/fusion/debug/weighted_error
                      ──/fusion/debug/imu_confidence
                      ──/fusion/debug/yaw_bias
                      ──/fusion/debug/yaw_measurement
vehicle_state_node    ──/vehicle/state────▶ tf2_publisher_node, trajectory_controller_node, behavior_planner_node, drift_estimator_node, bfmc_ui_node
                      ──/fusion/imu_confidence──▶ trajectory_controller_node, behavior_planner_node
tf2_publisher_node    ──/tf────────────────▶ (TF tree consumers)
```

### System / UI

```
runtime_telemetry_node ──/system/runtime_health──▶ bfmc_ui_node, health_monitor_node (optional)
health_monitor_node   ──/system/health/status, /system/health/metrics, /healthz──▶ (optional)
bfmc_ui_node ──/ui/status────▶ rosbridge_websocket ──▶ browser dashboard
bfmc_ui_server_node ──HTTP :8080──▶ browser (serves Bfmc_userinterface/dist + dashboard.html)
rosbridge_websocket ──WS :9090──▶ browser
```

## Service graph

No `create_service<...>` calls are present in the C++ source. The system uses topics only.

## Parameter graph (race-day, observed)

The race-day launch pins parameters explicitly (per the launch-file comment, to defeat code-default drift). Key tunables:

| Node | Parameter | Value | Purpose |
|---|---|---|---|
| image_preprocessor_node | `stale_threshold_s` | 0.50 | Tolerate USB / RealSense jitter. |
| lane_geometry_node | `stale_threshold_s` | 0.50 | Same. |
| free_space_node | `min_period_s` | 0.10 | Cap to 10 Hz. |
| object_detection_node | `min_period_s` | 0.125 | Cap to 8 Hz. |
| global_path_planner_node | `disable_pub` | True | Defer `/planning/global_path` to mission supervisor. |
| global_path_planner_node | `replan_consecutive_samples`, `replan_min_speed_mps`, `replan_min_interval_s` | 5, 0.10, 3.0 | Replan-storm guard. |
| mission_semantics_node | `supervisor_hz` | 10.0 | Mission tick rate. |
| mission_semantics_node | `mission_yaml` | `competition_mission_expanded.yaml` | BFS-expanded 420-node mission. |
| mission_semantics_node | `semantics_yaml` | `competition_semantics_expanded.yaml` | Inheritance-tagged semantics. |
| local_path_planner | `planner_hz` | 20.0 | Local-planner rate. |
| local_path_planner | `lookahead_m`, `num_points` | 2.0, 40 | Lookahead and resolution. |
| local_path_planner | `a_max`, `a_lat_max` | 1.8, 0.8 | Acceleration limits. |
| behavior_planner_node | `tick_period_s` | 0.20 | 5 Hz FSM. |
| behavior_planner_node | `degraded_speed`, `degraded_persist_ticks` | 0.20, 10 | Soft-degradation policy (10 ticks = 2.0 s). |
| behavior_planner_node | `pedestrian_confirm_s` / `clear_s` | 0.15 / 1.00 | Detection hysteresis. |
| behavior_planner_node | `city_speed`, `highway_speed`, `turn_speed`, `roundabout_speed`, `crosswalk_speed`, `parking_speed` | 0.30, 0.55, 0.22, 0.18, 0.12, 0.12 | Race-day speed table. |
| trajectory_controller_node | `control_rate_hz` | 20.0 | Control loop. |
| trajectory_controller_node | `abs_max_throttle`, `abs_max_brake` | 0.35, 0.5 | Output ceiling. |
| safety_supervisor_node | `forward_max`, `reverse_max`, `max_brake` | 0.35, 0.0, 0.5 | Hard clamp. |
| emergency_stop_node | `emergency_persist_ticks` | 5 | Soft-signal debounce. |
| system_state_machine_node | `ready_enter_samples`, `ready_exit_samples` | 20, 50 | 10 s assert, 25 s drop. |
| bfmc_ui_node | `publish_hz` | 5.0 | UI status rate. |
| bfmc_ui_server_node | `port` | 8080 | HTTP server. |
| rosbridge_websocket | `port`, `address` | 9090, `0.0.0.0` | WebSocket bridge. |

## Launch graph

Five launch files. Each composes a different subset of the same packages.

| Launch file | Purpose |
|---|---|
| `race_day.launch.py` | Full deployed stack. Reference launch. |
| `hil_bringup.launch.py` | Hardware-in-the-loop. Most race-day nodes minus UI. Adds `localization_client_node`. |
| `lane_follow.launch.py` | Lane-follow demo (no mission supervisor, no global planner). |
| `manual_drive.launch.py` | Teleop. Adds `teleop_to_control_node`. No behavior planner, no mission supervisor. |
| `manual_data_collection.launch.py` | Minimal — drivers + perception only, for data collection. |

## Deployment graph

```
Operator browser (Chrome / Firefox on LAN)
        │ HTTP :8080                 │ WS :9090
        ▼                            ▼
bfmc_ui_server_node              rosbridge_websocket
        │                            ▲
        ▼                            │
   Static dist (HTML/JS/CSS)    /ui/status JSON
                                     ▲
                                     │
                                bfmc_ui_node
                                     ▲
                                     │ (consumes ~20 topics)
                                     │
   ┌─────────────────────────────────┴──────────────────┐
   │                Autonomy stack (native)              │
   │  CycloneDDS / ROS_DOMAIN_ID=42                      │
   │                                                     │
   │   perception ⇄ planning ⇄ control                   │
   │       fusion ⇄ safety ⇄ TF                          │
   └─────────────────────────────────────────────────────┘
                  │                            ▲
                  ▼                            │
   ┌─────────────────────────────────────────────────────┐
   │ Docker: bfmc_drivers (host network, privileged)     │
   │                                                     │
   │   realsense2_camera (/camera/camera/*)              │
   │   micro_ros_agent (UART /dev/ttyACM0 @ 460800)      │
   └─────────────────────────────────────────────────────┘
                  │                            ▲
                  ▼                            │
              USB cable                     UART
                  │                            │
                  ▼                            ▼
              RealSense                     STM32F401RE
              D435i                         (vehicle firmware)
                                              │
                                              ▼
                                            Servo (PA0)
                                            ESC (PA1)
                                            BNO085 IMU
                                            Ultrasonic
```

## What this overview does not claim

- The Jetson stack uses standard ROS 2 patterns (topics + composable nodes + lifecycle). It does **not** use ROS 2 services or actions in any first-party node.
- The mission supervisor relies on YAMLs installed to `share/mission_supervisor_cpp/config/` and a GraphML installed to `share/global_path_planner_cpp/maps/`. The race-day launch passes those paths as parameters; when no parameter is supplied, the node resolves the canonical paths via `ament_index_cpp::get_package_share_directory(...)`.
- The `localization_client_node` is launched at race-day with `server_host=''`. Whether it is effectively idle in that configuration is **not proven** in this overview — it is read off the launch file only.
- Object-detection rates the model at 8 Hz (`min_period_s=0.125`), but the actual achieved rate depends on engine build, GPU thermal, and image size, and is not measured here.

Subsequent subsystem documents (`docs/subsystems/ros2/`, `docs/subsystems/docker/`, `docs/subsystems/micro_ros/`, `docs/subsystems/vehicle_pipeline/`) drill into each layer.
