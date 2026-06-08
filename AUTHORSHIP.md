# Authorship and Attribution

## First-party content

| Item | Author | License |
|---|---|---|
| All packages under `bfmc_msgs/`, `control/`, `drivers/`, `fusion/`, `perception/{free_space_cpp,image_preprocessor_cpp,lane_geometry_cpp,object_detection_cpp,object_tracker_cpp}/`, `planning/{behavior_planner_cpp,global_path_planner_cpp,local_path_planner_cpp,mission_supervisor_cpp,trajectory_generator}/`, `safety/`, `system/`, `system_bringup/`, `tools/` | Team OdyssiX | Apache-2.0 |
| Documentation under `docs/` and the root `README.md`, `PROJECT_KNOWLEDGE_INDEX.md`, `CONTRIBUTING.md`, `SECURITY.md`, `CHANGELOG.md` | Team OdyssiX | Apache-2.0 |
| Configuration files under `planning/config/` (mission, semantics, speed profiles, risk zones, behavior overrides) | Team OdyssiX | Apache-2.0 |
| Track graph and zones (`planning/global_path_planner_cpp/maps/`) | Team OdyssiX, derived from BFMC competition specification | Apache-2.0 |

## Third-party content

### Bosch / BFMC organizers reference UI (`bfmc_ui/`)

`bfmc_ui/` carries the Bosch Future Mobility Challenge organizers' reference "Brain" project user interface. License: BSD-3-Clause, Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers. The upstream license text is preserved verbatim at `bfmc_ui/LICENSE`.

This release ships only the load-bearing portion of the upstream subtree:

- `bfmc_ui/Bfmc_userinterface/dist/` — the pre-built React bundle. Served at runtime by `bfmc_ui_server_node` (port 8080).
- `bfmc_ui/LICENSE` — upstream license text.
- `bfmc_ui/README.md` — upstream description.

The upstream Raspberry Pi-side Python sources (`bfmc_ui/bin/`, `bfmc_ui/src/`, `bfmc_ui/test/`, `bfmc_ui/main.py`, `bfmc_ui/requirements_*.txt`) are not redistributed here because they are not invoked by the Jetson deployment. They remain available from the upstream Bosch BFMC documentation site referenced in `bfmc_ui/README.md`.

### Apt-installed ROS 2 packages

The deployed launch depends on the following packages installed via the standard ROS 2 Humble apt repository. They retain their upstream licenses:

| Package | License |
|---|---|
| `nav2_lifecycle_manager` | Apache-2.0 |
| `rclcpp_components` | Apache-2.0 |
| `rosbridge_server` | BSD-3-Clause |
| `tf2_ros` | Apache-2.0 |
| `tf2_web_republisher` (optional) | BSD-3-Clause |
| `realsense2_camera` | Apache-2.0 |
| `micro_ros_agent` (built from source in the drivers container) | Apache-2.0 |
| `rmw_cyclonedds_cpp` | EPL-2.0 |

### CycloneDDS (used as the RMW)

CycloneDDS is selected as the ROS 2 middleware implementation via `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. License: Eclipse Public License 2.0. Source: Eclipse CycloneDDS upstream.

## Companion repository

The OdyssiX vehicle firmware that pairs with this Brain repository — `BoschFutureMobilityChallenge2026-Embedded` — is authored by Team OdyssiX and licensed under Apache-2.0. It uses upstream FreeRTOS (MIT) and STM32 HAL / CMSIS (BSD-3-Clause / Apache-2.0). See that repository's `AUTHORSHIP.md` for detail.

## Inventory of release-required files

| File | Present | Source |
|---|---|---|
| `LICENSE` | Yes | Apache License, Version 2.0, prefixed with `Copyright 2026 OdyssiX Team` |
| `README.md` | Yes | First-party |
| `PROJECT_KNOWLEDGE_INDEX.md` | Yes | First-party |
| `AUTHORSHIP.md` | Yes | This document |
| `CONTRIBUTING.md` | Yes | First-party |
| `SECURITY.md` | Yes | First-party |
| `CHANGELOG.md` | Yes | First-party |
| `.gitignore` | Yes | First-party |
| `bfmc_ui/LICENSE` | Yes | Upstream Bosch / BFMC organizers — preserved verbatim |
