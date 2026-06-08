# Path Sanitization Report

Generated 2026-06-08. Phase A of the public-release hardening cycle.

## Scope

Every hard-coded user-specific path in the public repository must be eliminated from source code (`*.cpp`, `*.hpp`, `*.py`, `*.yaml`, `*.launch.py`, `*.sh`). Documentation may still discuss historical references; runtime artefacts may not depend on them.

## Findings

| # | File | Line | Original | Why it exists | Risk | Replacement strategy |
|---|---|---|---|---|---|---|
| F-1 | `planning/mission_supervisor_cpp/src/mission_semantics_node.cpp` | 29 | `/home/odyssix/bfmc_ws/install/global_path_planner_cpp/share/.../Competition_track_graph.graphml` | Parameter fallback for `graphml_path` if launch does not supply it. | New operator running the node without the wrapper launch hits a path that does not exist. Leaks maintainer home-dir name. | Replace with `ament_index_cpp::get_package_share_directory("global_path_planner_cpp") + "/maps/Competition_track_graph.graphml"` so the node resolves the path from the installed share tree on any machine. |
| F-2 | `planning/mission_supervisor_cpp/src/mission_semantics_node.cpp` | 33 | `/home/odyssix/bfmc_ws/install/mission_supervisor_cpp/share/mission_supervisor_cpp/config` | Parameter fallback for `config_dir`. | Same as F-1. | Same idiom: `ament_index_cpp::get_package_share_directory("mission_supervisor_cpp") + "/config"`. |
| F-3 | `perception/object_detection_cpp/src/object_detection_node.cpp` | 147 | `/home/odyssix/bfmc_ws/src/bfmc_autonomy/perception/object_detection_cpp/config/object_detection.yaml` | Last-resort fallback for `config_path` after parameter + env var. | Same as F-1. Additionally points into the source tree, not the installed tree — fragile. | `ament_index_cpp::get_package_share_directory("object_detection_cpp") + "/config/object_detection.yaml"`. |
| F-4 | `perception/object_detection_cpp/config/object_detection.yaml` | 8 | `engine_path: /home/odyssix/bfmc_models/object_detection/yolov8n_fp16.engine` | Operator's local TensorRT engine path. | Will not load on any other machine. | Set to empty string; add a node parameter `engine_path` that overrides the YAML value. Document the operator-set requirement in `README.md` and `docs/release/path_sanitization_report.md`. |
| F-5 | `perception/object_detection_cpp/scripts/probe_engine.sh` | 8 | `ENGINE="${1:-/home/odyssix/bfmc_models/object_detection/yolov8n_fp16.engine}"` | Convenience default for the script. | Same as F-4. | Remove the default; require the user to pass the engine path as `$1`. Emit a clear usage error if missing. |
| F-6 | `system/v2x_bridge_cpp/include/v2x_bridge_cpp/v2x_bridge_node.hpp` | 21 | Comment: `// UDP wire format (per /home/odyssix/v2x/udpListener.py):` | Comment cross-reference to maintainer's V2X protocol document. | Leaks maintainer home-dir name. Reader cannot follow the reference. | Replace with a generic description of where the protocol spec lives; remove the home-dir path. |
| F-7 | `system/v2x_bridge_cpp/include/v2x_bridge_cpp/v2x_bridge_node.hpp` | 28 | Comment: `// TCP wire format (per /home/odyssix/v2x/tcpClient.py):` | Same. | Same. | Same. |
| F-8 | `planning/global_path_planner_cpp/maps/zones.yaml` | 94 | Comment: `# UDP:5007 (see /home/odyssix/v2x/udpListener.py for protocol).` | Same. | Same. | Same. |
| F-9 | `system_bringup/launch/race_day.launch.py` | 31, 40, 42, 44 | Uses `os.path.expanduser('~/bfmc_ws/install/...')` | Workspace-name assumption — works for any user with workspace named `bfmc_ws`. | Operator using a differently-named workspace breaks the launch. Not strictly a personal path, but a workstation assumption. | Replace with `ament_index_python.packages.get_package_share_directory(...)` so the launch resolves the path from the installed share tree. |
| F-10 | `system_bringup/launch/hil_bringup.launch.py` | 192, 193, 222, 223 | Same `~/bfmc_ws/install/...` pattern | Same. | Same. | Same. |
| F-11 | `system_bringup/launch/lane_follow.launch.py` | 31 | Same `~/bfmc_ws/install/...` pattern | Same. | Same. | Same. |
| F-12 | `system_bringup/launch/manual_drive.launch.py` | 30 | Same `~/bfmc_ws/install/...` pattern | Same. | Same. | Same. |

## Non-findings (false positives)

The following were flagged by the scan but are correct as-is:

| Item | Reason it stays |
|---|---|
| `docker/docker-compose.yml:10` — `~/bfmc_ws:/bfmc_ws` | Docker compose bind-mount with `~` expansion; works for any user. The container-side path is fixed; the host-side path is user-resolved. This is the standard pattern. |
| Project-identity strings (`Team OdyssiX`, `OdyssiX competition mission`, etc.) | Project identity, not personal info. Preserved. |
| `system_bringup/setup.py:19` — `maintainer='Team OdyssiX'` | Project identity. |
| `planning/config/competition_mission.yaml:19` — `name: "BFMC 2026 — OdyssiX competition mission"` | Mission display name. Project identity. |

## Replacement plan

### F-1, F-2 — `mission_semantics_node.cpp`

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>

// ...
graphml_path_ = declare_parameter<std::string>(
    "graphml_path",
    ament_index_cpp::get_package_share_directory("global_path_planner_cpp")
        + "/maps/Competition_track_graph.graphml");
config_dir_  = declare_parameter<std::string>(
    "config_dir",
    ament_index_cpp::get_package_share_directory("mission_supervisor_cpp")
        + "/config");
```

`mission_supervisor_cpp/package.xml` already depends on `ament_index_cpp` transitively via `rclcpp`. No new declared dependency required.

### F-3 — `object_detection_node.cpp`

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>

// ...
if (config_path.empty()) {
    config_path = ament_index_cpp::get_package_share_directory("object_detection_cpp")
                  + "/config/object_detection.yaml";
}
```

### F-4 — `object_detection.yaml` + node parameter override

YAML becomes:

```yaml
model:
  # MUST be set by the operator. Either:
  #   (a) Replace this empty string with the absolute path to the engine, or
  #   (b) Pass `-p engine_path:=/abs/path/to/engine.engine` to the node.
  engine_path: ""
```

Node gains a parameter that overrides the YAML value:

```cpp
declare_parameter<std::string>("engine_path", "");
const std::string engine_override = get_parameter("engine_path").as_string();
if (!engine_override.empty()) {
    config_.model.engine_path = engine_override;
}
if (config_.model.engine_path.empty()) {
    throw std::runtime_error(
        "object_detection_node: engine_path must be set (YAML or -p engine_path:=...)");
}
```

### F-5 — `probe_engine.sh`

```bash
ENGINE="${1:-}"
if [[ -z "$ENGINE" ]]; then
  echo "usage: $0 <engine_path>" >&2
  exit 1
fi
```

### F-6, F-7, F-8 — comment rewrites

Replace the `/home/odyssix/v2x/...` references with generic descriptions that point at the protocol structure rather than the maintainer's filesystem.

### F-9, F-10, F-11, F-12 — launch files

Replace:

```python
graph_path = os.path.expanduser(
    '~/bfmc_ws/install/global_path_planner_cpp/share/global_path_planner_cpp/maps/Competition_track_graph.graphml')
```

with:

```python
from ament_index_python.packages import get_package_share_directory
graph_path = os.path.join(
    get_package_share_directory('global_path_planner_cpp'),
    'maps', 'Competition_track_graph.graphml')
```

Equivalent for `zones.yaml`, `mission_config_dir`, and the CycloneDDS profile.

## Application

Replacements applied in this Phase A pass. Post-fix verification scan included at the bottom of this document.

## Post-fix verification

Run from the public repo root:

```sh
# No /home/* in any source file:
! grep -rEn '/home/[a-zA-Z0-9_-]+' --include='*.cpp' --include='*.hpp' --include='*.py' \
    --include='*.cu' --include='*.yaml' --include='*.yml' --include='*.sh' \
    --include='*.launch.py' --include='*.cfg' .  && echo "PASS"

# No 'bfmc_ws' literal in any source file:
! grep -rEn 'bfmc_ws' --include='*.cpp' --include='*.hpp' --include='*.py' \
    --include='*.cu' --include='*.yaml' --include='*.yml' --include='*.sh' \
    --include='*.launch.py' --include='*.cfg' .  && echo "PASS"

# Docker compose still uses ~ (intentional):
grep -n '~/bfmc_ws' docker/docker-compose.yml  # one match expected
```

The first two checks are release-blocking. The third is informational — Docker compose bind-mounts are not in scope.
