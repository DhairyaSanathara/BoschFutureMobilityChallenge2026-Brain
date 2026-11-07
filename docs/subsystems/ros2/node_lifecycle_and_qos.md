# Node Lifecycle, Executors, Callback Groups, and QoS

Generated 2026-06-08. Grounded in source headers and node implementations.

## Lifecycle nodes (8 total)

The following nodes are `LifecycleNode` instances. The `nav2_lifecycle_manager` (`lifecycle_manager_autonomy`) auto-activates all 8 with `autostart=True`, `bond_timeout=0.0`, `attempt_respawn_reconnection=False`, 2 s after `trajectory_controller_node` spawns.

| Node | Package | Why lifecycle |
|---|---|---|
| `lane_geometry_node` | `lane_geometry_cpp` | Composable, in `perception_container`. Lifecycle separates configuration from activation. |
| `free_space_node` | `free_space_cpp` | Composable. Same reason. |
| `ekf_localization_node` | `ekf_localization` | Loads state model and IMU bias at configure-time; activates only when EKF is ready. |
| `drift_estimator_node` | `drift_estimator` | Activates after EKF so it has a state stream to align to. |
| `vehicle_state_node` | `vehicle_state` | Activates after EKF + drift estimator. |
| `tf2_publisher_node` | `tf2_publisher` | Activates after vehicle_state so it has a state stream to broadcast. |
| `behavior_planner_node` | `behavior_planner_cpp` | Activates after the upstream perception/planning chain. |
| `trajectory_controller_node` | `bfmc_control` | Activates last; only emits `/control/command_raw` after activation. |

`image_preprocessor_node` is composable but **not** lifecycle in the deployed launch. It is a plain `rclcpp_components::ComposableNode` instance inside `perception_container`.

## Composable container

```
perception_container (rclcpp_components::component_container_mt)
├── image_preprocessor_node     (plain composable, intra-process comms)
├── lane_geometry_node          (lifecycle composable, intra-process comms)
└── free_space_node             (lifecycle composable, intra-process comms)
```

`use_intra_process_comms=True` is set on every composable node description. This enables zero-copy pointer passing between the three composed nodes for the image stream. The standalone `object_detection_node` and `object_tracker_node` do not use intra-process comms (they live in separate processes).

Container settings: `respawn=True`, `respawn_delay=2.0`.

## Non-lifecycle nodes

These nodes are plain `rclcpp::Node` instances:

| Node | Package | Why not lifecycle |
|---|---|---|
| `imu_driver_node` | `imu_driver` | Driver. Streams whenever the upstream micro-ROS topic is alive. |
| `localization_client_node` | `ekf_localization` | TCP client; lifecycle would not add value. |
| `runtime_telemetry_node` | `runtime_telemetry_cpp` | Pure telemetry. |
| `image_preprocessor_node` | `image_preprocessor_cpp` | Composable but not lifecycle in deployed launch. |
| `object_detection_node` | `object_detection_cpp` | Has its own internal warm-up (TensorRT engine load). |
| `object_tracker_node` | `object_tracker_cpp` | Stateless processing. |
| `global_path_planner_node` | `global_path_planner_cpp` | Race-day runs with `disable_pub=True` so it stays "warm" but inactive. |
| `mission_semantics_node` | `mission_supervisor_cpp` | Owns mission state; runs at 10 Hz independent of lifecycle. |
| `v2x_bridge_node` | `v2x_bridge_cpp` | UDP listener. |
| `local_path_planner_node` | `local_path_planner_cpp` | 20 Hz planner. |
| `trajectory_generator_node` | `trajectory_generator` | Stateless. |
| `safety_supervisor_node`, `emergency_stop_node`, `system_state_machine_node`, `watchdog_node` | `safety` | Must always be running, especially during lifecycle transitions of the autonomy chain. |
| `bfmc_ui_node`, `bfmc_ui_server_node` | `system/*` | UI surface. |
| `health_monitor_node` | `health_monitor_cpp` | Optional; off by default. |

## Executors

All C++ nodes use the default `rclcpp::executors::SingleThreadedExecutor` unless the launch file specifies a composable container. The composable container `perception_container` uses `component_container_mt` which is the multithreaded variant — it gives each composed node its own thread for callbacks, so the image preprocessor → lane geometry → free space chain can pipeline.

The deployed race-day pattern is:

- `perception_container` (multithreaded)
- Every other node: single-threaded, single executor

No `MultiThreadedExecutor` is instantiated explicitly anywhere in first-party source.

## Callback groups

The deployed source uses ROS 2 default callback groups. Searches across the C++ source for explicit `rclcpp::CallbackGroup::Type::Reentrant` or `MutuallyExclusive` constructions return no first-party usage. The defaults apply:

- One mutually-exclusive callback group per node by default.
- Composable nodes inside `component_container_mt` get one thread each from the executor pool, which gives concurrency across nodes; within a node, callbacks still serialise.

## QoS choices (observed)

Observations from `create_publisher<...>` / `create_subscription<...>` arguments in the C++ source:

| Topic / category | QoS pattern |
|---|---|
| Image / depth from RealSense (`/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`) | `SensorDataQoS()` (best-effort, KeepLast). Standard for camera streams. |
| Lane geometry, free space, preprocessed image | Default Reliable, KeepLast(10). |
| Control commands (`/control/command_raw`, `/control/command`) | Default Reliable, KeepLast(10). The supervisor sits between raw and final; QoS is reliable because every command counts. |
| Emergency / state topics (`/safety/*`, `/system/*`) | Latched-style: `TransientLocal` durability on the state machine output. |
| UI status (`/ui/status`) | Default Reliable, KeepLast. Bridged to rosbridge for the browser. |
| TF (`/tf`) | TF default (Reliable KeepLast). |
| Micro-ROS topic (`/microros/imu_raw`) | Reliable (defaults). Upstream micro-ROS Agent maps DDS to RTPS over UART. |

Explicit QoS profile constructions vary per node; the spec deliberately does not enumerate every QoS depth here because it is read off literal declarations and not a unified policy document.

## Topic-name conventions

- `/camera/camera/*` — RealSense double-namespace (from `rs_launch.py` default).
- `/microros/*` — STM32 stream via the micro-ROS agent (`/microros/imu_raw`). The `_hardware` remap arguments in docker-compose do not change the name the Brain subscribes to; see `docs/subsystems/docker/docker_audit.md`.
- `/perception/*` — perception layer output.
- `/planning/*` — planner layer output. Sub-namespace `/planning/debug/*` for development topics.
- `/control/*` — control output. Sub-namespace `/control/debug/*` for development.
- `/safety/*` — safety triggers.
- `/system/*` — system state and runtime telemetry.
- `/vehicle/*` — fused vehicle state.
- `/fusion/*` — fusion-layer debug + corrections.
- `/v2x/*` — vehicle-to-everything bridge.
- `/ui/*` — UI surface.

No topic name in the first-party source crosses these conventions.

## Node dependency graph (build-time)

```
bfmc_msgs
   │
   ├──▶ used by every C++ planning/perception/control package
   │
   ├──▶ bfmc_control
   ├──▶ behavior_planner_cpp
   ├──▶ local_path_planner_cpp
   ├──▶ global_path_planner_cpp
   ├──▶ mission_supervisor_cpp
   ├──▶ trajectory_generator
   ├──▶ object_detection_cpp
   ├──▶ object_tracker_cpp
   ├──▶ free_space_cpp
   ├──▶ lane_geometry_cpp
   ├──▶ image_preprocessor_cpp
   ├──▶ drift_estimator
   ├──▶ ekf_localization
   ├──▶ tf2_publisher
   ├──▶ vehicle_state
   ├──▶ safety
   ├──▶ bfmc_ui_cpp
   ├──▶ runtime_telemetry_cpp
   ├──▶ v2x_bridge_cpp
   └──▶ health_monitor_cpp
```

No first-party package depends on another first-party package besides `bfmc_msgs`. The whole stack is wired through topic interfaces, not direct package dependencies.

## Topic dependency graph (runtime, simplified)

```
camera/color/image_raw ──▶ image_preprocessor ──▶ lane_geometry ──┐
                       └──────────────────────▶ object_detection ─▶ object_tracker
camera/depth/image_rect_raw ─────────────────▶ free_space         │
                                                                  ▼
                                            local_path_planner ◄──┘
                                            global_path_planner ─▶ (race_day: idle)
                                            mission_semantics ──▶ /planning/global_path
                                            local_path_planner ──▶ /planning/local_path
                                            trajectory_generator ─▶ /planning/trajectory ──▶ trajectory_controller ──▶ /control/command_raw
                                            behavior_planner ──▶ /planning/target_speed ──▶ trajectory_controller
                                            safety_supervisor ──▶ /control/command ──▶ micro_ros_agent ──▶ STM32

   STM32 ──▶ micro_ros_agent ──▶ /microros/imu_raw ───────────▶ imu_driver ──▶ /imu/data_filtered ──▶ ekf_localization
                                                                                                       ──▶ drift_estimator
                                                                                                       ──▶ vehicle_state ──▶ /vehicle/state ──▶ tf2_publisher
                                                                                                                                                   ──▶ behavior_planner
                                                                                                                                                   ──▶ trajectory_controller
```

The fusion chain is independent of the perception chain. They converge at `behavior_planner` (which reads both) and at `trajectory_controller` (which reads vehicle state for feedforward control).
