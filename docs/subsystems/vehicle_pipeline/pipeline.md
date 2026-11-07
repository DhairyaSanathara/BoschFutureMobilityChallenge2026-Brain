# Vehicle Pipeline (Deployed)

Generated 2026-06-08. This document describes what actually ran on race day. Aspirational architecture and superseded prototypes are explicitly out of scope.

## End-to-end pipeline

```
RealSense D435i  ──┐
                   │ /camera/camera/color/image_raw  (15 Hz, sensor QoS)
                   │ /camera/camera/depth/image_rect_raw
                   │ /camera/camera/color/camera_info
                   ▼
        image_preprocessor_node (CUDA, composable)
                   │ /perception/image_preprocessed
                   ▼
   ┌───────────────┼───────────────────────────────┐
   │               │                               │
   ▼               ▼                               ▼
lane_geometry  free_space (10 Hz cap)        object_detection (8 Hz cap)
   │               │                               │
   │ /perception   │ /perception                   │ /perception
   │ /lane_geom    │ /free_space                   │ /objects
   │               │                               ▼
   │               │                          object_tracker
   │               │                               │ /perception
   │               │                               │ /tracked_objects
   ▼               ▼                               ▼
       local_path_planner_node (20 Hz)
                   │ /planning/local_path
                   │ /planning/path_confidence
                   ▼
         trajectory_generator_node
                   │ /planning/trajectory
                   ▼
        trajectory_controller_node (lifecycle, 20 Hz)
                   │ /control/command_raw
                   ▼
            safety_supervisor_node (clamp + saturate)
                   │ /control/command
                   ▼
            micro_ros_agent (UART, 460800 baud)
                   ▼
                  STM32 firmware
                   │
                   ▼
               PWM out → steering servo + Quicrun ESC
```

Two side flows feed the pipeline:

```
STM32 ──UART──▶ micro_ros_agent ──/microros/imu_raw───────────▶ imu_driver_node
                                                                  │ /imu/data_filtered
                                                                  ▼
                                                       ekf_localization_node (lifecycle)
                                                                  │ /vehicle/state_raw
                                                                  ▼
                                                       vehicle_state_node (lifecycle)
                                                                  │ /vehicle/state
                                                                  ▼
                                              tf2_publisher_node (broadcasts odom→base_link)

mission_supervisor_cpp::mission_semantics_node (10 Hz)
   │ /planning/global_path        (BFS-expanded 420-node mission)
   │ /planning/current_region     (e.g. crosswalk, roundabout, highway)
   │ /planning/current_speed_limit
   │ /navigation/flags
   ▼
behavior_planner_node (lifecycle, 5 Hz FSM)
   │ /planning/target_speed
   │ /planning/lateral_offset
   │ /planning/behavior_state
   ▼
trajectory_controller_node
```

The `drift_estimator_node` (lifecycle) closes a yaw correction loop using `/perception/lane_geometry`:

```
lane_geometry ──▶ drift_estimator ──/fusion/yaw_bias_correction──▶ vehicle_state
```

## Stage-by-stage detail

### Stage 1 — Sensor (RealSense + IMU + Ultrasonic)

| Source | Topic | Rate | Notes |
|---|---|---|---|
| RealSense D435i color | `/camera/camera/color/image_raw` | 15 Hz | Set in `docker/realsense_params.yaml`. |
| RealSense D435i depth | `/camera/camera/depth/image_rect_raw` | 15 Hz | Same. |
| BNO085 (via STM32) | `/microros/imu_raw` | ~100 Hz batched | STM32 polls IMU @ 1 kHz, batches publish ~10 Hz cadence per packet (10–20 messages per second observed). |
| Ultrasonic front (via STM32) | `/microros/ultrasonic_front` | Not published | Current firmware has no ultrasonic publisher; `ultrasonic_front_driver` is not in any launch. |

### Stage 2 — Preprocess + perception

- `image_preprocessor_node` (CUDA preprocess) → 15 Hz pass-through.
- `lane_geometry_node` is callback-driven; inherits 15 Hz from upstream.
- `free_space_node` caps itself with `min_period_s=0.10` → 10 Hz.
- `object_detection_node` caps itself with `min_period_s=0.125` → 8 Hz. TensorRT engine builds on first run from an ONNX model; race-day notes assume the engine is already cached on the device.
- `object_tracker_node` consumes detection output → publishes tracked objects.

### Stage 3 — Localization + fusion

- `ekf_localization_node` (lifecycle) integrates `/imu/data_filtered` (and optionally external localization client output if `server_host` is set; race-day runs with it empty → effectively IMU-only EKF) → publishes `/vehicle/state_raw`.
- `drift_estimator_node` (lifecycle) reads `/perception/lane_geometry` + `/vehicle/state` → publishes `/fusion/yaw_bias_correction`. `enable_debug_topics` is gated on `enable_runtime_validation`. Stale thresholds are 0.50 s on both state and lane inputs.
- `vehicle_state_node` (lifecycle) fuses `/vehicle/state_raw` with `/fusion/yaw_bias_correction` → publishes `/vehicle/state` + `/fusion/imu_confidence`.
- `tf2_publisher_node` (lifecycle) broadcasts `odom→base_link` from `/vehicle/state`.

### Stage 4 — Mission orchestration

- `mission_semantics_node` (10 Hz) loads the GraphML track graph + 5 YAMLs (mission_expanded, semantics_expanded, speed_profiles, risk_zones, behavior_overrides). It publishes per-node region tags, speed limits, current-node tracking, and navigation flags.
- `global_path_planner_node` is held in memory with `disable_pub=True` — it does not publish the global path during race day (the mission supervisor does).
- `v2x_bridge_node` listens for UDP on port 5007 → publishes `/v2x/cars`, `/v2x/semaphores`. TCP is disabled (`enable_tcp=False`).

### Stage 5 — Planning

- `local_path_planner_node` (20 Hz) consumes the global path + lane geometry + free space + tracked objects + vehicle state → publishes `/planning/local_path` and `/planning/path_confidence`.
- `behavior_planner_node` (lifecycle, 5 Hz) is the FSM. Wide subscription fan-in (see ros2/packages.md). Publishes target speed, lateral offset, behavior state, plus a heavy debug suite. Race-day pins ~30 parameters including the speed table (city 0.30, highway 0.55, turn 0.22, roundabout 0.18, crosswalk 0.12, parking 0.12) and detection hysteresis.
- `trajectory_generator_node` converts local path + target speed → timed trajectory.

### Stage 6 — Control

- `trajectory_controller_node` (lifecycle, 20 Hz) runs the path-follower. Race-day clamps: `abs_max_throttle=0.35`, `abs_max_brake=0.5`. Goal radius 0.25 m. Publishes `/control/command_raw`.
- `safety_supervisor_node` clamps `/control/command_raw` against the supervisor limits → publishes `/control/command`.
- `/control/command` is bridged via the micro-ROS agent → STM32 firmware → PWM.

### Stage 7 — Safety wrap

Always running, never lifecycle-gated:

- `emergency_stop_node` aggregates emergency triggers (5-tick debounce for soft signals, immediate for hard signals like manual / watchdog / vehicle_ready=false).
- `system_state_machine_node` ticks at 2 Hz; READY enter requires 20 valid samples (10 s), exit takes 50 invalid (25 s).
- `watchdog_node` is deferred 10 s after launch; then it monitors `/planning/trajectory` for stalls.

## Timing summary (deployed)

| Loop | Rate | Why |
|---|---|---|
| Camera (color+depth) | 15 Hz | Bandwidth + USB jitter ceiling. |
| Free space | 10 Hz | Down-throttled with `min_period_s=0.10`. |
| Object detection | 8 Hz | TensorRT throughput at chosen image size + class set. |
| Mission supervisor | 10 Hz | `supervisor_hz=10.0`. |
| Behavior planner | 5 Hz | `tick_period_s=0.20`. |
| Local path planner | 20 Hz | `planner_hz=20.0`. |
| Trajectory controller | 20 Hz | `control_rate_hz=20.0`. |
| System state machine | 2 Hz | `tick_period_s=0.50`. |
| UI status | 5 Hz | `publish_hz=5.0`. |
| Telemetry (proc/thermal) | 1 Hz (metrics) / 5 Hz (summary) | Defaults in runtime_telemetry. |

## Safety chain (engineering)

```
  /control/command_raw  (from trajectory_controller_node)
            │
            ▼
   safety_supervisor_node
        forward_max=0.35      reverse_max=0.0
        deadband=0.05         max_brake=0.5
            │
            ▼
        /control/command  ──▶ micro_ros_agent ──▶ STM32
                                                   │
                                                   ▼
                                         500 ms deadman
                                         → neutral PWM if silent
```

Hard-stop triggers (instant emergency):

- `manual_emergency` (operator stop button)
- `watchdog_alarm` (trajectory stream stall)
- `vehicle_ready=false` (state machine drops READY)
- Lifecycle deactivation of `trajectory_controller_node`

Soft-stop triggers (5 ticks ≈ 1 s debounce at 5 Hz):

- Degraded confidence on lane / localization
- Emergency reasons from upstream subsystems

The STM32 firmware applies its own 500 ms command-silence deadman: if `/control/command` stops arriving (e.g. micro-ROS agent crashed, USB unplug), the firmware returns PWM to 1500 µs neutral within 500 ms.

## What is NOT in the deployed pipeline

The following were either prototyped, planned, or partially built — but **did not run** under `race_day.launch.py`:

| Item | Status | Where in repo |
|---|---|---|
| Python prototypes (`image_preprocessor`, `behavior_planner`, `local_path_planner`) | Superseded by their `_cpp` counterparts | Not redistributed in this release |
| `clock_sync_node` | Built but never launched | `drivers/clock_sync_node/` |
| `ultrasonic_front_driver` | Built but never launched | `drivers/ultrasonic_front_driver/` |
| `health_monitor_node` | Built, off by default | gated behind `enable_runtime_validation:=true` |
| `tf2_web_republisher` | Optional | launch silently skips if not installed |
| `v2x_bridge_node` TCP path | Disabled (`enable_tcp=False`) | UDP-only at race day |

## What this document does not claim

- This document does not assert any *behavioural performance* numbers (lap time, lane-tracking RMS, intervention rate). Those would belong in a race-report document.
- The CUDA pipeline ('preprocess_cuda.cu` in image_preprocessor_cpp and object_detection_cpp) is described as "CUDA-accelerated" only; the actual kernels (resize / colour space conversion / NCHW packing) are not enumerated here.
- The TensorRT engine path is treated as opaque. The engine itself is built on-device and not checked into the repository — see `perception/object_detection_cpp/scripts/probe_engine.sh`.
