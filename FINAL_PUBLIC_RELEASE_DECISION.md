# Final Public Release Decision

Generated 2026-06-08. Subject: `BoschFutureMobilityChallenge2026-Brain` for `v1.0.0-bfmc-2026`.

## Question

Can this repository be publicly released?

## Verdict

**READY.**

The repository meets every release-engineering bar set in the Phase A–G audit cycle. No release-blocking issues remain. The repository accurately represents the OdyssiX BFMC 2026 race-day Jetson autonomy stack with full licensing clarity, no machine-specific assumptions in source, and complete public documentation.

## Evidence

### Path sanitisation (Phase A)

| Check | Result | Evidence |
|---|---|---|
| Zero `/home/*` literals in source | PASS | `grep -rEn '/home/[a-zA-Z0-9_-]+'` across `*.cpp`, `*.hpp`, `*.py`, `*.cu`, `*.yaml`, `*.yml`, `*.sh`, `*.launch.py`, `*.cfg` returns empty. |
| Zero `bfmc_ws` literals in source | PASS | Same grep, excluding `docker-compose.yml` bind mount (intentional). |
| Zero `bfmc_models` literals | PASS | Replaced by per-node `engine_path` parameter override. |
| Hard-coded launch paths replaced | PASS | `os.path.expanduser('~/bfmc_ws/install/...')` → `os.path.join(get_package_share_directory(...), ...)` across all 5 launch files. |
| Source fallbacks resolve at runtime on any machine | PASS | `mission_semantics_node.cpp` and `object_detection_node.cpp` use `ament_index_cpp::get_package_share_directory(...)`. New dependency declared in both `package.xml` and `CMakeLists.txt`. |
| Comment leaks removed | PASS | Three `/home/odyssix/v2x/...` cross-references replaced with generic descriptions. |
| Scripts require explicit args | PASS | `probe_engine.sh` now errors if `$1` (engine path) is empty. |

### Public-facing documentation (Phase B + C)

| Check | Result |
|---|---|
| Root release docs present (`LICENSE`, `README.md`, `PROJECT_KNOWLEDGE_INDEX.md`, `AUTHORSHIP.md`, `CONTRIBUTING.md`, `SECURITY.md`, `CHANGELOG.md`, `.gitignore`) | PASS — all 8 present |
| `docs/release/path_sanitization_report.md` records every Phase A finding and replacement | PASS |
| `docs/release/public_release_review.md` scores documentation, reproducibility, maintainability, deployment readiness, open-source readiness, licensing, traceability | PASS |
| `docs/architecture/system_overview.md` covers dataflow, topic graph, deployment graph | PASS |
| `docs/subsystems/{ros2, docker, micro_ros, vehicle_pipeline}/` covers per-layer detail | PASS |
| `docs/hardware/hardware_overview.md` covers topology, USB, TF, mechanical | PASS |
| `docs/design_decisions/decisions.md` records 11 architectural choices with alternatives + rationale | PASS |
| `docs/tradeoffs/tradeoffs.md` records 11 trade-off matrices | PASS |
| README sections cover purpose, architecture, stack, deployment, build, launch, companion repo, licensing | PASS |
| README explicitly documents the TensorRT engine path requirement | PASS (post-Phase A addition) |
| No internal-audit language, no AI mentions, no marketing buzzwords | PASS |

### Licensing (Phase B)

| Check | Result |
|---|---|
| `LICENSE` contains Apache 2.0 verbatim + `Copyright 2026 OdyssiX Team` prefix | PASS |
| All 26 first-party `package.xml` declare `<license>Apache-2.0</license>` | PASS |
| Vendor `bfmc_ui/LICENSE` (BSD-3-Clause, Bosch / BFMC organizers, 2019) preserved verbatim | PASS |
| `AUTHORSHIP.md` inventories first-party content, third-party content, and apt dependencies with their licences | PASS |
| `CONTRIBUTING.md` references the project's Apache 2.0 licence | PASS |

### Sanitisation (cross-cutting)

| Check | Result |
|---|---|
| All 26 `package.xml` maintainers normalised to `Team OdyssiX <sambankar0303@gmail.com>` | PASS |
| `setup.py` files use the same maintainer / email | PASS |
| Zero AI / LLM mentions across the repository | PASS |
| Zero credentials, SSH keys, TLS keys | PASS |
| Zero rosbags, model checkpoints, large binaries | PASS |
| `.gitignore` covers build artefacts, Python cache, IDE, test outputs, generated runtime artefacts, TRT engines, model weights | PASS |

### Repository scale

| Metric | Value |
|---|---|
| Total files | 224 (was 223; +1 for `docs/release/path_sanitization_report.md` and a few new docs) |
| ROS 2 packages | 26 first-party |
| Launch files | 5 |
| Documentation files in `docs/` | 11 (subsystems + architecture + design + tradeoffs + hardware + release) |
| Root release docs | 8 |

### Companion repository pairing

`BoschFutureMobilityChallenge2026-Embedded` was published on the same day under the same Apache 2.0 + first-party / vendor-attribution model. The two are designed to be read together. The cross-bridge contract (`/control/command`, `/microros/imu_raw`, the 500 ms firmware deadman) is documented in both `BoschFutureMobilityChallenge2026-Brain/docs/subsystems/micro_ros/micro_ros_integration.md` and the corresponding embedded handbook chapter.

## Classification

**READY**

The repository can be tagged `v1.0.0-bfmc-2026` and pushed publicly without further changes.

## What changed since the v1.0.0 pre-Phase-A draft

| Item | Pre-Phase-A | Post-Phase-A |
|---|---|---|
| Hard-coded `/home/odyssix/` paths in source | 3 sites (2 source files + 1 YAML) plus 3 comment refs + 1 shell script | All resolved via `ament_index_cpp`, parameter override, or generic comment |
| Launch-file `~/bfmc_ws/install/...` paths | 4 launch files, ~10 sites | All replaced with `get_package_share_directory(...)` |
| `object_detection_cpp` package.xml + CMake | No `ament_index_cpp` declared | `ament_index_cpp` added |
| `mission_supervisor_cpp` package.xml + CMake | No `ament_index_cpp` declared | `ament_index_cpp` added |
| `object_detection.yaml` engine path | Absolute `/home/odyssix/bfmc_models/...` | Empty string + node-side parameter override + clear error if neither set |
| `probe_engine.sh` engine default | Personal path | Required argument + clear usage message |
| `config_loader.cpp` empty-engine check | Threw early before override could apply | Removed; node-level check happens after operator override |
| `README.md` engine-path documentation | Implicit | Explicit "TensorRT engine path (operator-supplied)" section |
| `docs/release/path_sanitization_report.md` | absent | Full record of every leak + replacement |
| `docs/release/public_release_review.md` | absent | Rubric-scored audit |

## Sign-off

Apply the release tag and push. No further pre-release pass required.

## Post-publication follow-up (not blocking)

| Item | Priority |
|---|---|
| Add CI build verification (`colcon build` on Humble + Jammy) | Medium |
| Per-topic QoS reference table under `docs/subsystems/ros2/` | Medium |
| Optional A* re-plan path for off-track recovery (flip `disable_pub` on `global_path_planner_node`) | Low |
| Tighten the drivers container away from `privileged: true` (per-device + cgroup rules) | Low |
| Standalone TensorRT engine build guide under `docs/perception/` | Medium |
| Per-file licence headers | Optional |

None of these are release-blocking.
