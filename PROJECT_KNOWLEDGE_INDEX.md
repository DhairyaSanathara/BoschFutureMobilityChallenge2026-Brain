# Project Knowledge Index

A starting-point guide for readers who want to find the answer to a specific question.

## Read in this order

1. `README.md` — what the project is, how to build it, how to launch it.
2. `docs/architecture/system_overview.md` — end-to-end dataflow, topic graph, deployment graph.
3. `docs/subsystems/vehicle_pipeline/pipeline.md` — stage-by-stage of what runs at race day.
4. `docs/subsystems/ros2/packages.md` — per-package detail.
5. `docs/design_decisions/decisions.md` — why the architecture is the way it is.

## Find by question

| Question | Document |
|---|---|
| What packages are in the repo and what do they do? | `docs/subsystems/ros2/packages.md` |
| What's the topic graph? | `docs/architecture/system_overview.md` § Topic graph |
| Which nodes are lifecycle? | `docs/subsystems/ros2/node_lifecycle_and_qos.md` |
| How does the Brain talk to the STM32? | `docs/subsystems/micro_ros/micro_ros_integration.md` |
| What's in the Docker container? | `docs/subsystems/docker/docker_audit.md` |
| What hardware is on the vehicle? | `docs/hardware/hardware_overview.md` |
| Why ROS 2 Humble? Why Docker? Why micro-ROS? | `docs/design_decisions/decisions.md` |
| Why C++ for everything deployed? | `docs/design_decisions/decisions.md` § 8 |
| Why pin every race-day parameter in the launch file? | `docs/design_decisions/decisions.md` § 10 |
| Trade-off matrices | `docs/tradeoffs/tradeoffs.md` |
| Where is the safety boundary? | `SECURITY.md` |
| Who authored what? | `AUTHORSHIP.md` |

## Find by subsystem

| Subsystem | Source path | Documentation |
|---|---|---|
| Messages | `bfmc_msgs/` | `docs/subsystems/ros2/packages.md` § bfmc_msgs |
| Drivers (IMU) | `drivers/imu_driver/` | `docs/subsystems/ros2/packages.md` § imu_driver |
| Perception (image preprocess, lane, free space) | `perception/{image_preprocessor_cpp,lane_geometry_cpp,free_space_cpp}/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 2 |
| Perception (object detection, tracker) | `perception/{object_detection_cpp,object_tracker_cpp}/` | Same |
| Fusion | `fusion/{ekf_localization,drift_estimator,vehicle_state,tf2_publisher}/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 3 |
| Mission orchestration | `planning/mission_supervisor_cpp/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 4 |
| Planning | `planning/{global_path_planner_cpp,local_path_planner_cpp,behavior_planner_cpp,trajectory_generator}/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 5 |
| Control | `control/trajectory_controller/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 6 |
| Safety | `safety/` | `docs/subsystems/vehicle_pipeline/pipeline.md` § Stage 7 |
| UI | `system/{bfmc_ui_cpp,bfmc_ui_server}/` + `bfmc_ui/Bfmc_userinterface/dist/` | `docs/subsystems/ros2/packages.md` § UI |
| Telemetry | `system/{runtime_telemetry_cpp,health_monitor_cpp}/` | `docs/subsystems/ros2/packages.md` § Telemetry |
| V2X | `system/v2x_bridge_cpp/` | `docs/subsystems/ros2/packages.md` § v2x_bridge_cpp |
| Bringup | `system_bringup/` | `docs/architecture/system_overview.md` § Launch graph |

## Find by topic name

The topic graph is enumerated in `docs/architecture/system_overview.md` § Topic graph and `docs/subsystems/ros2/node_lifecycle_and_qos.md` § Topic dependency graph.

## Find by deployment artefact

| Artefact | Why it ships | Reference |
|---|---|---|
| `planning/config/competition_mission.yaml` | Operator-authored 236-node mission | `mission_supervisor_cpp` |
| `planning/config/competition_mission_expanded.yaml` | BFS gap-filled to 420 nodes — consumed at runtime by `mission_semantics_node` | race_day launch line 160 |
| `planning/config/competition_semantics.yaml` | Operator-authored per-node region tags | inheritance input |
| `planning/config/competition_semantics_expanded.yaml` | Inheritance-tagged — consumed at runtime by `mission_semantics_node` | race_day launch line 161 |
| `planning/config/competition_{behavior_overrides,risk_zones,speed_profiles}.yaml` | Region dispatch tables | `mission_supervisor_cpp` parameters |
| `planning/global_path_planner_cpp/maps/Competition_track_graph.graphml` | BFMC competition track graph | global planner + mission supervisor |
| `planning/global_path_planner_cpp/maps/zones.yaml` | Zone overlay | global planner |
| `planning/global_path_planner_cpp/maps/Track2025_checkpoints_fINAL.png` | Track reference image | visual reference only |
| `system_bringup/config/cyclonedds_race.xml` | Optional LAN-only CycloneDDS profile | not active in race_day launch |
| `perception/object_detection_cpp/config/object_detection.yaml` | Detector class table, NMS thresholds | object_detection_node |
| `docker/realsense_params.yaml` | RealSense launch params | mounted into drivers container |

## What is NOT in this repository

- The TensorRT engine file used by `object_detection_node` — built on-device from an ONNX model. See `perception/object_detection_cpp/scripts/`.
- Trained model weights — not redistributed here.
- Rosbags — not checked in.
- Mechanical drawings, wiring harnesses — owned by OdyssiX hardware build documentation, separate.
- Race-day operations log / mission reports — internal to the team.
