# Design Decisions

Generated 2026-06-08. Each decision is recorded as: the choice, the alternatives considered, the trade-off, and the reason for selection. None of these decisions are retro-fitted; they are documented from the deployed state.

## 1. ROS 2 Humble as the framework

**Choice.** ROS 2 Humble Hawksbill (LTS).

**Alternatives considered.**

- ROS 1 Noetic — last ROS 1 release.
- Custom C++ middleware (e.g., direct DDS, gRPC, ZMQ).
- ROS 2 Iron / Jazzy — newer ROS 2 releases.

**Trade-off.**

| Aspect | ROS 1 Noetic | Custom mw | ROS 2 Humble | ROS 2 Iron/Jazzy |
|---|---|---|---|---|
| LTS | Yes (May 2025 EOL) | n/a | Yes (May 2027 EOL) | Iron non-LTS; Jazzy LTS until 2029 |
| micro-ROS | Limited | Build it | First-class | First-class |
| RealSense driver | Mature | DIY | Mature | Mature |
| Composable nodes | No | DIY | Yes | Yes |
| nav2 lifecycle manager | No | DIY | Yes | Yes |
| Maturity at race-day time (mid-2026) | Frozen | n/a | Mature | Iron stale, Jazzy newer but JetPack 6 still defaults to 22.04 + Humble |

**Reason selected.** Humble is the natural pair with Ubuntu 22.04 / JetPack 6.x. micro-ROS for Humble is well-supported. Lifecycle manager, composable nodes, intra-process comms — all needed by the deployed stack — are mature in Humble. ROS 1 was rejected because micro-ROS does not target it.

## 2. CycloneDDS as the RMW

**Choice.** `rmw_cyclonedds_cpp`. Both the autonomy stack and the docker container set this explicitly via `SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')` and `ENV` in the Dockerfile.

**Alternatives considered.**

- `rmw_fastrtps_cpp` (Fast DDS) — default in Humble.
- `rmw_connextdds` (Connext) — commercial.

**Trade-off.**

| Aspect | CycloneDDS | Fast DDS | Connext |
|---|---|---|---|
| License | Eclipse Public License | Apache | Commercial |
| Multicast discovery | Robust | Robust | Robust |
| Behavior under intra-host high-rate image traffic | Empirically good for our race-day setup | Mixed reports | Excellent |
| micro-ROS compatibility | Standard | Standard | Standard |

**Reason selected.** CycloneDDS is the operator's chosen pairing because of its observed stability under the docker-on-host + native autonomy stack pattern at race-day speeds. A previously-attempted LAN-only Cyclone profile (`cyclonedds_race.xml`) bottlenecked the color image stream; reverting to stock multicast resolved it. The XML file ships for reference.

## 3. Docker (driver container) — but not for autonomy

**Choice.** Run RealSense + micro-ROS agent inside a `bfmc_drivers` container; run the autonomy stack natively on the Jetson.

**Alternatives considered.**

- Everything in one big container.
- Everything native (no Docker).
- Multiple containers (one per subsystem).

**Trade-off.**

| Aspect | All-in-container | All-native | Drivers-in-container (chosen) | Many containers |
|---|---|---|---|---|
| Reproducibility of camera + agent | Good | Painful (rosdep + APT pinning) | Good | Good |
| TensorRT / CUDA access | Requires nvidia-runtime + CUDA libs in image (large) | Native; uses Jetson's CUDA install directly | Native | Each container needs CUDA |
| Build time | Large | None | Small image | Many large images |
| Inter-process latency | Same DDS hop | Same | Same (host network) | Same |
| Operational complexity | Single docker compose | Multiple terminals | Two layers (compose + ros2 launch) | Many layers |

**Reason selected.** The driver layer benefits from container isolation (RealSense + micro-ROS agent install reproducibly). The autonomy layer needs direct CUDA / TensorRT access on the Jetson and benefits from running native to skip the container-runtime overhead and avoid pinning a CUDA toolchain inside the image. `network_mode: host` keeps DDS multicast trivial between the two.

## 4. micro-ROS over UART (not Ethernet)

**Choice.** XRCE-DDS over USART2 at 460 800 baud.

**Alternatives considered.**

- micro-ROS over UDP (Ethernet on a host MCU).
- Direct serial protocol (custom framing).
- CAN bus.

**Trade-off.**

| Aspect | UART micro-ROS | UDP micro-ROS | Custom serial | CAN |
|---|---|---|---|---|
| Hardware required on STM32F401 | UART (built-in) | LAN8742 PHY or equivalent (not on Nucleo) | UART | CAN peripheral + transceiver |
| Maintenance cost | Standard micro-ROS | Same | Custom forever | Custom forever |
| Reliability | Reliable by default | Reliable | DIY | Reliable |
| Bandwidth | 460 800 baud (~46 kB/s effective) | 100 Mbit | Same as UART | 1 Mbit |
| Race-day need | IMU @ 10 Hz batched + control @ 20 Hz | Same | Same | Same |

**Reason selected.** The deployed system's micro-ROS traffic is dominated by IMU publish (small) and a 20 Hz control command (small). 460 800 baud has ~10× headroom. The STM32F401RE has no Ethernet PHY, so UDP would require an external module. Custom serial loses years of micro-ROS work. CAN was rejected because the BFMC chassis doesn't expose CAN to the vehicle controller.

## 5. Jetson Orin Nano as the compute

**Choice.** NVIDIA Jetson Orin Nano running Ubuntu 22.04 + JetPack 6.x.

**Alternatives considered.**

- Raspberry Pi 5 + Coral / Hailo accelerator.
- Jetson Nano (original).
- NUC + USB GPU.

**Trade-off.**

| Aspect | Orin Nano | RPi5 + accel | Jetson Nano | NUC + GPU |
|---|---|---|---|---|
| CUDA / TensorRT first-class | Yes | No (different stack) | Yes (older) | Yes but bigger/heavier |
| Power budget | ~7–15 W | ~5–10 W | ~5–10 W | 25 W+ |
| Form factor on 1/10 chassis | Fits | Fits | Fits | Tight |
| ROS 2 Humble availability | First-class | First-class | First-class but unsupported JetPack | First-class |
| Cost (relative) | Mid | Low | Low (EOL) | High |

**Reason selected.** TensorRT throughput on the Orin Nano is sufficient for the deployed object-detection rate (8 Hz). JetPack 6.x ships Ubuntu 22.04, the natural Humble base. Power budget is compatible with the BFMC battery pack. The previous-gen Jetson Nano (5W mode) was insufficient for the YOLO inference rate.

## 6. Single composable container for the image chain

**Choice.** `perception_container` is a `rclcpp_components::component_container_mt` hosting `image_preprocessor_node` + `lane_geometry_node` + `free_space_node` with `use_intra_process_comms=True`.

**Alternatives considered.**

- Three separate processes.
- One monolithic perception node.

**Trade-off.**

- Three processes: serialise the image on each topic hop → expensive copy + DDS overhead.
- Monolithic node: hard to debug, hard to lifecycle independently.
- Composable container: zero-copy pointer passing intra-process; per-node lifecycle; clean separation of concerns.

**Reason selected.** Zero-copy + per-node lifecycle is exactly the use case `rclcpp_components` was designed for. The multithreaded variant (`*_mt`) lets the three composed nodes execute on independent threads, so preprocessing for frame N can overlap with lane extraction for frame N-1.

## 7. nav2 lifecycle manager for the autonomy chain

**Choice.** `nav2_lifecycle_manager::lifecycle_manager` with `autostart=True`, deferred 2 s after `trajectory_controller_node` spawns, managing 8 lifecycle nodes.

**Alternatives considered.**

- A custom orchestrator.
- No lifecycle manager — each node self-activates on construction.

**Trade-off.**

- Custom orchestrator: more code, more failure modes.
- No manager: each node racing to activate without coordination; bond reporting still needed for health.
- nav2 manager: battle-tested, supports auto-respawn-toggle and bond-timeout settings; off-the-shelf.

**Reason selected.** Re-implementing what nav2 has spent years polishing is not a good use of race-day engineering effort. The deferred-trigger pattern (2 s after the controller spawns) lets the upstream nodes settle before activation.

## 8. C++ for deployed perception/planning/control; Python prototypes archived

**Choice.** Every deployed node is C++. The Python equivalents (image_preprocessor, behavior_planner, local_path_planner) are kept in-tree as superseded prototypes but never launched.

**Alternatives considered.**

- All-Python deployed stack.
- C++ deployed, Python deleted entirely.

**Trade-off.**

- All-Python: simpler iteration but slower (especially perception + planning at 20 Hz).
- C++ + delete Python: cleaner repo but loses the design provenance.
- C++ + Python prototypes archived (chosen): the Python prototypes were validated first; the C++ deployed binaries replaced them. The Python prototypes are not redistributed in this public release.

**Reason selected.** The Python prototypes hit the iteration speed we needed during early development, but could not hit the 20 Hz behavior + 20 Hz controller rates the deployed stack requires. We rewrote in C++ once the interfaces stabilised.

## 9. Mission supervisor owns `/planning/global_path` (not the global planner)

**Choice.** `mission_semantics_node` publishes `/planning/global_path`. `global_path_planner_node` runs with `disable_pub=True`.

**Alternatives considered.**

- Global planner publishes `/planning/global_path`; mission supervisor only emits annotations.
- One node combining mission + global plan.

**Trade-off.**

- Global planner owns path: mission supervisor has to synchronise on the planner's tick rate.
- Combined node: mixes the BFMC-specific mission (graph node sequence) with the A* primitive.
- Mission supervisor owns path (chosen): the mission graph is a fixed BFS-expanded sequence; the supervisor cycles through it at 10 Hz and stamps every node with its region tag. The global planner remains warm in case it's needed for re-plan.

**Reason selected.** The BFMC mission is more naturally expressed as a fixed sequence of graph nodes (with semantics) than as an A* query. The mission supervisor reads the expanded YAML directly and emits the path one cycle at a time. The global planner stays in the launch graph for diagnostics and for non-race scenarios.

## 10. Race-day speeds pinned in the launch file

**Choice.** The race-day launch file pins every speed parameter explicitly (`city_speed=0.30`, `highway_speed=0.55`, `crosswalk_speed=0.12`, `parking_speed=0.12`, …).

**Alternatives considered.**

- Rely on code defaults.
- Use a single `race_day.yaml` parameter file.

**Trade-off.**

- Code defaults: silent regression if a future PR changes a default.
- Separate YAML file: another file to drift.
- Launch-file pinning (chosen): the launch file is the deployment artefact; every parameter visible on one screen.

**Reason selected.** The launch-file comment is explicit: *"Pinning every P1–P4 tunable explicitly so a code-default drift cannot silently re-introduce a regression."* That's the whole argument.

## 11. Apache-2.0 for first-party content

**Choice.** Apache License 2.0 for OdyssiX-authored code and documentation.

**Alternatives considered.** MIT, BSD-3-Clause, Mozilla MPL-2.0.

**Trade-off.** Apache 2.0 has an explicit patent grant and is compatible with the BSD-3-Clause `bfmc_ui/` upstream and the MIT FreeRTOS used by the embedded repo.

**Reason selected.** Consistency with the OdyssiX embedded firmware release. Aligns with the ROS 2 ecosystem's most common license. The `bfmc_msgs` package already declares Apache-2.0; one `package.xml` declares MIT (will be reconciled in Phase 9).
