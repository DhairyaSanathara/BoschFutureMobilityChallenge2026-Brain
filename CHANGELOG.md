# Changelog

All notable changes to the BoschFutureMobilityChallenge2026-Brain repository.

This project follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0-bfmc-2026] — 2026-06-08

Initial public release of the OdyssiX Jetson autonomy stack from the Bosch Future Mobility Challenge 2026.

### Added

- ROS 2 Humble autonomy stack for the OdyssiX BFMC 2026 vehicle.
- 26 first-party ROS 2 packages spanning perception, planning, control, fusion, safety, and UI.
- Drivers Docker container (RealSense + micro-ROS agent built from source for ARM64 Jetson).
- Five launch files: `race_day`, `hil_bringup`, `lane_follow`, `manual_drive`, `manual_data_collection`.
- Mission orchestration based on the BFMC competition track graph (GraphML) and BFS-expanded mission semantics.
- Documentation tree under `docs/` (architecture, subsystems, design decisions, trade-offs, hardware, release).
- Bosch / BFMC organizers reference UI (`bfmc_ui/Bfmc_userinterface/dist/`) served by `bfmc_ui_server_node` on port 8080.

### Changed

- `mission_supervisor_cpp::mission_semantics_node`: parameter fallbacks for `graphml_path` and `config_dir` now resolve via `ament_index_cpp::get_package_share_directory(...)` (previously contained workstation-specific paths).
- `object_detection_cpp::object_detection_node`: `config_path` fallback resolves via `ament_index_cpp::get_package_share_directory(...)`; added an `engine_path` node parameter that overrides the YAML value so the same configuration ships to any machine.
- `perception/object_detection_cpp/config/object_detection.yaml`: `model.engine_path` is now empty; operator must set it either by editing the YAML or by passing `-p engine_path:=/abs/path/to/engine`.
- `perception/object_detection_cpp/src/config_loader.cpp`: removed the early empty-engine_path throw; the check now lives in the node after the parameter override.
- All five launch files under `system_bringup/launch/`: `os.path.expanduser('~/bfmc_ws/install/...')` patterns replaced with `ament_index_python.packages.get_package_share_directory(...)` so the launch resolves package paths independent of workspace name.
- `perception/object_detection_cpp/scripts/probe_engine.sh`: removed the workstation-specific default for the engine argument; the script now exits with a usage message if no path is supplied.
- `system/v2x_bridge_cpp/include/v2x_bridge_cpp/v2x_bridge_node.hpp` and `planning/global_path_planner_cpp/maps/zones.yaml`: replaced comment cross-references that pointed at the maintainer's local filesystem with generic descriptions of the JSON wire format.
- `mission_supervisor_cpp` and `object_detection_cpp`: added explicit `ament_index_cpp` dependency in both `package.xml` and `CMakeLists.txt`.
- Every `package.xml` and `setup.py` maintainer field normalised to `Team OdyssiX <sambankar0303@gmail.com>`.
- Every first-party `package.xml` licence field normalised to `Apache-2.0`.

### Notes

- Three Python prototype packages (`image_preprocessor`, `behavior_planner`, `local_path_planner`) were superseded by their C++ counterparts before deployment and are not part of this release.
- The upstream Bosch RPi Python sources under `bfmc_ui/{bin,src,test}/` are not redistributed here because they are not invoked by the Jetson deployment; the load-bearing `Bfmc_userinterface/dist/` bundle is shipped under BSD-3-Clause attribution.
- The optional CycloneDDS race-day profile (`system_bringup/config/cyclonedds_race.xml`) ships but is not active in `race_day.launch.py`; stock multicast is used for race-day robustness.
