# ROS 2 Packages

Generated 2026-06-08. 26 first-party packages ship in this release. Build system: `ament_cmake` (C++) and `ament_python` (Python launch host).

Three additional Python prototype packages (`image_preprocessor`, `behavior_planner`, `local_path_planner`) existed during development. They were superseded by their `_cpp` counterparts before deployment and are not redistributed here.

## Package roster

| # | Package | Type | Deployed (race_day) | Path | Executables |
|---|---|---|---|---|---|
| 1 | `bfmc_msgs` | ament_cmake | yes | `bfmc_msgs/` | (interfaces only) |
| 2 | `bfmc_control` | ament_cmake | yes | `control/trajectory_controller/` | `trajectory_controller_node`, `teleop_to_control_node` |
| 3 | `clock_sync_node` | ament_cmake | no | `drivers/clock_sync_node/` | `clock_sync_node` |
| 4 | `imu_driver` | ament_cmake | yes | `drivers/imu_driver/` | `imu_driver_node` |
| 5 | `ultrasonic_front_driver` | ament_cmake | no | `drivers/ultrasonic_front_driver/` | `ultrasonic_front_driver_node` |
| 6 | `drift_estimator` | ament_cmake | yes (lifecycle) | `fusion/drift_estimator/` | `drift_estimator_node` |
| 7 | `ekf_localization` | ament_cmake | yes (lifecycle for EKF; non-lifecycle for client) | `fusion/ekf_localization/` | `ekf_localization_node`, `localization_client_node` |
| 8 | `tf2_publisher` | ament_cmake | yes (lifecycle) | `fusion/tf2_publisher/` | `tf2_publisher_node` |
| 9 | `vehicle_state` | ament_cmake | yes (lifecycle) | `fusion/vehicle_state/` | `vehicle_state_node` |
| 10 | `free_space_cpp` | ament_cmake | yes (lifecycle, composable) | `perception/free_space_cpp/` | `free_space_node` |
| 11 | `image_preprocessor_cpp` | ament_cmake | yes (lifecycle, composable) | `perception/image_preprocessor_cpp/` | `image_preprocessor_node` (composable) |
| 12 | `lane_geometry_cpp` | ament_cmake | yes (lifecycle, composable) | `perception/lane_geometry_cpp/` | `lane_geometry_node` |
| 13 | `object_detection_cpp` | ament_cmake | yes | `perception/object_detection_cpp/` | `object_detection_node` |
| 14 | `object_tracker_cpp` | ament_cmake | yes | `perception/object_tracker_cpp/` | `object_tracker_node` |
| 15 | `behavior_planner_cpp` | ament_cmake | yes (lifecycle) | `planning/behavior_planner_cpp/` | `behavior_planner_node` |
| 16 | `global_path_planner_cpp` | ament_cmake | yes | `planning/global_path_planner_cpp/` | `global_path_planner_node` |
| 17 | `local_path_planner_cpp` | ament_cmake | yes | `planning/local_path_planner_cpp/` | `local_path_planner_node` |
| 18 | `mission_supervisor_cpp` | ament_cmake | yes | `planning/mission_supervisor_cpp/` | `mission_semantics_node`, `mission_expander`, `mission_semantics_inherit`, `mission_trace` |
| 19 | `trajectory_generator` | ament_cmake | yes | `planning/trajectory_generator/` | `trajectory_generator_node` |
| 20 | `safety` | ament_cmake | yes | `safety/` | `safety_supervisor_node`, `emergency_stop_node`, `system_state_machine_node`, `watchdog_node` |
| 21 | `bfmc_ui_cpp` | ament_cmake | yes | `system/bfmc_ui_cpp/` | `bfmc_ui_node` |
| 22 | `bfmc_ui_server` | ament_cmake | yes | `system/bfmc_ui_server/` | `bfmc_ui_server_node` |
| 23 | `health_monitor_cpp` | ament_cmake | optional (off by default) | `system/health_monitor_cpp/` | `health_monitor_node` |
| 24 | `runtime_telemetry_cpp` | ament_cmake | yes | `system/runtime_telemetry_cpp/` | `runtime_telemetry_node` |
| 25 | `v2x_bridge_cpp` | ament_cmake | yes | `system/v2x_bridge_cpp/` | `v2x_bridge_node` |
| 26 | `system_bringup` | ament_python | yes (launch host) | `system_bringup/` | launch files only |

External (apt-installed) ROS packages used at launch time: `nav2_lifecycle_manager`, `rclcpp_components`, `rosbridge_server`, `tf2_ros`, `tf2_web_republisher` (optional), `realsense2_camera`, `micro_ros_agent`, `rmw_cyclonedds_cpp`.

## Per-package detail

### `bfmc_msgs`

| Field | Value |
|---|---|
| Purpose | Frozen message contracts for the OdyssiX autonomy stack. |
| Messages | `ControlCommand`, `DetectedObject`, `ObjectArray`, `FreeSpace`, `LaneGeometry`, `Trajectory`, `VehicleState` |
| Publishers | none (interface-only) |
| Subscribers | none |
| Services | none |
| Build deps | `rosidl_default_generators`, `std_msgs`, `geometry_msgs`, `sensor_msgs` |
| Exec deps | `rosidl_default_runtime` |

### `bfmc_control` (path: `control/trajectory_controller/`)

| Field | Value |
|---|---|
| Purpose | Path-following control + supervisor-aware command publisher; teleop bridge. |
| Executables | `trajectory_controller_node` (lifecycle), `teleop_to_control_node` (non-lifecycle, manual_drive only) |
| Publishes | `/control/command_raw`, `/control/command` (when teleop_to_control bridges), `/control/debug/clamped_throttle`, `/control/debug/raw_throttle`, `/control/debug/supervisor_state` |
| Subscribes | `/planning/trajectory`, `/planning/behavior_state`, `/planning/target_speed`, `/planning/lateral_offset`, `/vehicle/state`, `/fusion/imu_confidence`, `/system/system_state`, `/system/vehicle_ready`, `/safety/emergency`, `/cmd_vel` (teleop only) |
| Dependencies | `rclcpp`, `rclcpp_lifecycle`, `bfmc_msgs`, `std_msgs`, `geometry_msgs` |

### `imu_driver`

| Field | Value |
|---|---|
| Purpose | Subscribe to micro-ROS IMU raw, republish filtered IMU. |
| Publishes | `/imu/data_filtered` |
| Subscribes | `/microros/imu_raw` (`imu_driver_node.hpp:40`) |
| Dependencies | `rclcpp`, `sensor_msgs` |

### `clock_sync_node` (built but not deployed)

| Field | Value |
|---|---|
| Purpose | Clock-sync helper. Source present, not referenced in any launch. |
| Publishes | `/clock_sync` |
| Status | EXPERIMENTAL / non-deployed |

### `ultrasonic_front_driver` (built but not deployed)

| Field | Value |
|---|---|
| Purpose | Front ultrasonic driver. Not referenced in any launch. |
| Publishes | `/ultrasonic/front` (`ultrasonic_front_driver_node.hpp:46`) |
| Subscribes | `/microros/ultrasonic_front` (`ultrasonic_front_driver_node.hpp:47`). The current firmware does not publish this topic. |
| Status | EXPERIMENTAL / non-deployed |

### `drift_estimator` (lifecycle)

| Field | Value |
|---|---|
| Purpose | Lane-anchored yaw drift correction. |
| Publishes | `/fusion/yaw_bias_correction`, `/fusion/debug/correction_active`, `/fusion/debug/curvature_weight`, `/fusion/debug/lane_heading`, `/fusion/debug/message_age_ms`, `/fusion/debug/node_latency_ms`, `/fusion/debug/weighted_error`, `/fusion/debug/imu_confidence`, `/fusion/debug/yaw_bias`, `/fusion/debug/yaw_measurement` |
| Subscribes | `/perception/lane_geometry`, `/vehicle/state` |
| Parameters | `stale_state_max_s=0.50`, `stale_lane_max_s=0.50`, `enable_debug_topics` |

### `ekf_localization`

| Field | Value |
|---|---|
| Purpose | EKF integrating IMU (+ optional external localization). |
| Executables | `ekf_localization_node` (lifecycle), `localization_client_node` (non-lifecycle TCP client) |
| Publishes | `/vehicle/state_raw`, `/localization/raw_position` (from client) |
| Subscribes | `/imu/data_filtered` (presumed via SubscriberFactory pattern; literal present in header) |
| Parameters (client) | `server_host`, `server_port`, `frame_id`, `reconnect_delay_s`, `recv_timeout_s` |

### `tf2_publisher` (lifecycle)

| Field | Value |
|---|---|
| Purpose | Publish `odom→base_link` TF from `/vehicle/state`. |
| Publishes | `/tf` |
| Subscribes | `/vehicle/state` |

### `vehicle_state` (lifecycle)

| Field | Value |
|---|---|
| Purpose | Fuse EKF raw state + drift correction → published vehicle state. |
| Publishes | `/vehicle/state`, `/fusion/imu_confidence` |
| Subscribes | `/vehicle/state_raw`, `/fusion/yaw_bias_correction` |

### `free_space_cpp` (lifecycle, composable)

| Field | Value |
|---|---|
| Purpose | Free-space (drivable region) mask from preprocessed image + depth. |
| Publishes | `/perception/free_space` |
| Subscribes | `/perception/image_preprocessed`, `/camera/camera/depth/image_rect_raw` |
| Parameters | `min_period_s=0.10`, `stale_threshold_s=0.50` |

### `image_preprocessor_cpp` (lifecycle, composable)

| Field | Value |
|---|---|
| Purpose | CUDA-accelerated image preprocess (resize / colour conversion). |
| Publishes | `/perception/image_preprocessed` |
| Subscribes | `/camera/camera/color/image_raw` |
| Parameters | `stale_threshold_s=0.50` |

### `lane_geometry_cpp` (lifecycle, composable)

| Field | Value |
|---|---|
| Purpose | Lane-line geometry extraction (polynomial fit). |
| Publishes | `/perception/lane_geometry` |
| Subscribes | `/perception/image_preprocessed` |
| Parameters | `stale_threshold_s=0.50` |

### `object_detection_cpp`

| Field | Value |
|---|---|
| Purpose | TensorRT YOLO detector + class table + NMS. |
| Publishes | `/perception/objects`, `/perception/debug/*` (per scripts) |
| Subscribes | `/camera/camera/color/image_raw` |
| Parameters | `min_period_s=0.125` (8 Hz), `stale_threshold_s=0.50` |
| External | TensorRT (CUDA). Engine built on-device. |
| Config | `config/object_detection.yaml` (classes, NMS, IoU thresholds). |

### `object_tracker_cpp`

| Field | Value |
|---|---|
| Purpose | ByteTrack + Kalman tracker for detections. |
| Publishes | `/perception/tracked_objects`, `/perception/debug/active_tracks`, `/perception/debug/id_switch_count`, `/perception/debug/track_count`, `/perception/debug/tracker_latency_ms` |
| Subscribes | `/perception/objects` |

### `behavior_planner_cpp` (lifecycle, 5 Hz)

| Field | Value |
|---|---|
| Purpose | Behavioral FSM. Soft-degradation, detection hysteresis, region speed table, overtake parameters. |
| Publishes | `/planning/behavior_state`, `/planning/target_speed`, `/planning/lateral_offset`, `/planning/current_behavior`, `/planning/mission_progress`, `/planning/mission_score_estimate`, `/planning/competition_path`, `/planning/competition_markers`, `/planning/debug/*` |
| Subscribes | `/planning/local_path`, `/planning/global_path`, `/planning/path_confidence`, `/planning/current_region`, `/planning/current_speed_limit`, `/planning/current_node`, `/planning/localization_confidence`, `/perception/tracked_objects`, `/perception/lane_geometry`, `/v2x/cars`, `/v2x/semaphores`, `/vehicle/state`, `/fusion/imu_confidence`, `/navigation/flags`, `/safety/emergency` |
| Tick | `tick_period_s=0.20` (5 Hz) |

### `global_path_planner_cpp`

| Field | Value |
|---|---|
| Purpose | A*/Dijkstra over `Competition_track_graph.graphml` with zone overlay. Race-day defers to mission supervisor via `disable_pub=True`. |
| Publishes | `/planning/global_path` (when `disable_pub=False`) |
| Subscribes | `/vehicle/state`, `/planning/goal_node` |

### `local_path_planner_cpp` (20 Hz)

| Field | Value |
|---|---|
| Purpose | Local path from `/planning/global_path` + `/perception/lane_geometry` + obstacle awareness. |
| Publishes | `/planning/local_path`, `/planning/path_confidence` |
| Subscribes | `/planning/global_path`, `/perception/lane_geometry`, `/perception/free_space`, `/perception/tracked_objects`, `/vehicle/state` |

### `mission_supervisor_cpp` (10 Hz)

| Field | Value |
|---|---|
| Purpose | Mission orchestration. Owns the global path expressed as a node sequence; emits per-node region tags, speed limits, behavior overrides, and navigation flags. |
| Executables | `mission_semantics_node` (main node), `mission_expander` (BFS path expander tool), `mission_semantics_inherit` (semantics inheritance tool), `mission_trace` (runtime trace dumper) |
| Publishes | `/planning/global_path`, `/navigation/flags`, `/planning/current_node`, `/planning/current_region`, `/planning/current_speed_limit`, `/planning/goal_node`, `/planning/localization_confidence` |
| Subscribes | `/vehicle/state`, `/v2x/semaphores`, `/v2x/cars` |
| Inputs | GraphML track graph + YAML mission/semantics/speed/risk/overrides. |

### `trajectory_generator`

| Field | Value |
|---|---|
| Purpose | Convert `/planning/local_path` to a timed `/planning/trajectory`. |
| Publishes | `/planning/trajectory` |
| Subscribes | `/planning/local_path`, `/planning/target_speed` |

### `safety` (four executables)

| Executable | Publishes | Subscribes | Role |
|---|---|---|---|
| `safety_supervisor_node` | `/control/command` | `/control/command_raw`, `/safety/emergency`, `/system/system_state` | Clamps control with `forward_max`, `reverse_max`, `deadband`, `max_brake`. |
| `emergency_stop_node` | `/safety/emergency`, `/safety/emergency_reason`, `/safety/emergency_stop` | `/safety/manual_emergency`, `/safety/watchdog_alarm`, `/system/vehicle_ready`, soft signals from supervisor/state | Aggregates emergency triggers with `emergency_persist_ticks` debounce. |
| `system_state_machine_node` | `/system/system_state`, `/system/vehicle_ready` | `/safety/emergency`, lifecycle bond signals | Tick 0.5 s. READY enter/exit hysteresis. |
| `watchdog_node` | `/safety/watchdog_alarm` | `/planning/trajectory` | Stall detection on the trajectory stream. |

### `bfmc_ui_cpp`

| Field | Value |
|---|---|
| Purpose | Aggregates a JSON status snapshot. |
| Publishes | `/ui/status` (std_msgs/String JSON) |
| Subscribes | wide fan-in across `/planning/*`, `/vehicle/state`, `/system/*`, `/safety/*` |
| Tick | `publish_hz=5.0` |

### `bfmc_ui_server`

| Field | Value |
|---|---|
| Purpose | HTTP file server on port 8080 serving the legacy Bosch React build + `dashboard.html`. |
| Publishes / Subscribes | none |
| External | static files installed to `share/bfmc_ui_server/web` + `share/bfmc_ui_server/Bfmc_userinterface/dist` |

### `health_monitor_cpp` (optional)

| Field | Value |
|---|---|
| Purpose | Topic-rate health monitor with metrics. Off by default in race_day. |
| Publishes | `/system/health/status`, `/system/health/metrics`, `/healthz` |
| Subscribes | wide fan-in on topic timestamps. |
| Parameters | `rate_window_s=5.0`, `metrics_period_s=1.0`, `summary_period_s=5.0` |

### `runtime_telemetry_cpp`

| Field | Value |
|---|---|
| Purpose | Jetson telemetry. Reads `/proc/stat`, `/proc/meminfo`, `/sys/class/thermal/*/temp`. |
| Publishes | `/system/runtime_health` |
| Subscribes | none |

### `v2x_bridge_cpp`

| Field | Value |
|---|---|
| Purpose | UDP receiver (port 5007) for V2X data with optional TCP. |
| Publishes | `/v2x/cars`, `/v2x/semaphores` |
| Subscribes | none |
| Parameters | `udp_port=5007`, `tcp_host=''`, `tcp_port=9000`, `locsys_id=0`, `tx_rate_hz=2.0`, `enable_tcp=False` |

### `system_bringup`

| Field | Value |
|---|---|
| Purpose | Launch host. Five launch files + CycloneDDS profile. |
| Launch files | `race_day.launch.py`, `hil_bringup.launch.py`, `lane_follow.launch.py`, `manual_drive.launch.py`, `manual_data_collection.launch.py` |
| Configs | `config/cyclonedds_race.xml` |

## Startup sequence (race-day)

Order of `Node` actions in `race_day.launch.py`:

1. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42` (env vars set)
2. Drivers: `imu_driver_node`, `localization_client_node`, `runtime_telemetry_node`
3. Perception: `perception_container` (composable), `object_detection_node`, `object_tracker_node`
4. Planning: `global_path_planner_node` (with `disable_pub=True`), `mission_semantics_node`, `v2x_bridge_node`, `local_path_planner_node`, `trajectory_generator_node`
5. Safety: `safety_supervisor_node`, `emergency_stop_node`, `system_state_machine_node`
6. Static TF: `static_tf_map_to_odom`, `static_tf_base_to_camera`, `static_tf_base_to_imu`
7. Fusion lifecycles: `ekf_localization_node`, `drift_estimator_node`, `vehicle_state_node`, `tf2_publisher_node`
8. Planner lifecycle: `behavior_planner_node`
9. Control lifecycle: `trajectory_controller_node`
10. UI: `bfmc_ui_node`, `rosbridge_websocket`, `bfmc_ui_server_node`
11. Optional: `health_monitor_node` (gated on `enable_runtime_validation`)
12. **2 s after `trajectory_controller_node` spawns** (via `OnProcessStart` + `TimerAction`): `lifecycle_manager_autonomy` → auto-activates the 8 lifecycle nodes.
13. **10 s after launch start**: `watchdog_node`.

The defer-by-time pattern (`TimerAction`) is used deliberately: the lifecycle manager waits for the control node to come up before issuing transitions, and the watchdog waits until the trajectory pipeline is producing valid frames before it starts checking for stalls.
