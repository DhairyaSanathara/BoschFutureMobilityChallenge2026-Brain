# Docker Subsystem

Generated 2026-06-08. Scope: `docker/` at the repository root. Three files. One container.

## Files

| File | Role |
|---|---|
| `docker/Dockerfile.drivers` | Image definition for `bfmc_drivers`. |
| `docker/docker-compose.yml` | Compose service definition. |
| `docker/realsense_params.yaml` | RealSense launch parameters, mounted into the container. |

## Image: `bfmc_drivers`

Built from `Dockerfile.drivers`. Base: `ros:humble`.

```
FROM ros:humble
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-can-msgs \
    can-utils \
    git python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    . /opt/ros/humble/setup.sh && \
    apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

WORKDIR /bfmc_ws
```

### What the image contains

- ROS 2 Humble base
- `realsense2_camera` apt package
- `cv_bridge` apt package
- `rmw_cyclonedds_cpp` apt package
- `can_msgs` apt package + `can-utils` (CAN tooling shipped but not exercised by deployed launch)
- Python 3 + pip + git
- micro-ROS agent built from source against Humble in `/microros_ws/` (this is the canonical way to get the agent on ARM64 Jetson, as the apt-published binary is x86-only at time of writing)

### Why micro-ROS is built from source, not apt

The comment in the Dockerfile says: *"Removed micro-ros-agent due to ARM64 limits"*. The host platform is the Jetson Orin Nano (ARM64). `ros-humble-micro-ros-agent` is unreliable / unavailable for ARM64 in apt repositories, so the image clones `micro_ros_setup` and runs `create_agent_ws.sh` + `build_agent.sh` to produce a local agent build.

### Environment variables baked into the image

| Variable | Value | Purpose |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Ensures every container child process uses CycloneDDS, matching the host autonomy stack. |
| `ROS_DOMAIN_ID` | `42` | Matches the host. Required for DDS discovery across container ↔ host. |

## Compose service: `drivers`

```
services:
  drivers:
    image: bfmc_drivers
    build:
      context: .
      dockerfile: Dockerfile.drivers
    network_mode: host
    privileged: true
    volumes:
      - ~/bfmc_ws:/bfmc_ws
      - /dev:/dev
      - ./realsense_params.yaml:/realsense_params.yaml
```

### Settings

| Setting | Value | Why |
|---|---|---|
| `network_mode: host` | shared host network namespace | DDS discovery between the host autonomy stack and the container's RealSense / micro-ROS topics. Avoids the multicast / NAT problems that bridge networking would introduce. |
| `privileged: true` | full device + capability access | Required for `/dev/ttyACM0` (STM32) and the RealSense USB device. Could be tightened with `--device` + cgroup rules, but the operator has chosen full privilege for race-day reliability. |
| `volumes: ~/bfmc_ws:/bfmc_ws` | bind-mount the host workspace | Lets the container see the operator's local launch / config files if needed. |
| `volumes: /dev:/dev` | bind-mount `/dev` | RealSense USB enumeration + STM32 ACM device. |
| `volumes: ./realsense_params.yaml:/realsense_params.yaml` | parameter file | Mounted in so changes don't require an image rebuild. |

### Startup command

```
bash -c "
source /opt/ros/humble/setup.bash
source /microros_ws/install/setup.bash

echo 'Starting RealSense D435i in the background...'
ros2 launch realsense2_camera rs_launch.py params_file:=/realsense_params.yaml &

echo 'Waiting for STM32 on /dev/ttyACM0...'
while [ ! -e /dev/ttyACM0 ]; do
  echo 'STM32 not found. Please plug it in or check connections...'
  sleep 2;
done

echo 'STM32 detected! Starting micro-ROS agent for physical hardware...'
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 460800 --ros-args \
  --remap /microros/imu_raw:=/microros/imu_raw_hardware \
  --remap /microros/ultrasonic_front:=/microros/ultrasonic_front_hardware
"
```

### Startup order

1. Source ROS 2 Humble overlay.
2. Source the micro-ROS workspace overlay (puts `micro_ros_agent` on the path).
3. Launch `realsense2_camera` in the background with the mounted params file.
4. Block until `/dev/ttyACM0` appears (with a 2 s polling loop).
5. Launch the micro-ROS agent on `/dev/ttyACM0` at 460800 baud.
6. The agent runs in the foreground; container exits if the agent exits.

### Topic remaps

The agent command passes two `--remap` arguments: `/microros/imu_raw` → `/microros/imu_raw_hardware` and `/microros/ultrasonic_front` → `/microros/ultrasonic_front_hardware`. Neither changes what the autonomy stack uses:

| STM32 publishes | Brain subscribes to | Evidence |
|---|---|---|
| `/microros/imu_raw` | `/microros/imu_raw` | `drivers/imu_driver/include/imu_driver/imu_driver_node.hpp:40`; `tools/check_run.sh` measures the IMU rate on the same name. |
| (nothing) | `/microros/ultrasonic_front` (`ultrasonic_front_driver`, not launched) | The current STM32 firmware has one publisher, `/microros/imu_raw`. |

The deployed `imu_driver_node` receives the STM32 stream on the unsuffixed name, so use `/microros/imu_raw` when inspecting it on the Jetson (`ros2 topic hz /microros/imu_raw`).

## Realsense params

`docker/realsense_params.yaml` — mounted at `/realsense_params.yaml` in the container, passed to `rs_launch.py` via `params_file:=`. Sets camera resolution, FPS, and topic enables. Race-day note: the camera FPS is set to **15 Hz** (referenced in the race-day launch comment).

## Volume mappings (summary)

| Host path | Container path | Direction | Purpose |
|---|---|---|---|
| `~/bfmc_ws` | `/bfmc_ws` | rw | Operator workspace (launch files, configs). |
| `/dev` | `/dev` | rw | USB device enumeration. |
| `./realsense_params.yaml` | `/realsense_params.yaml` | ro (effective) | Camera parameters. |

## Network configuration

`network_mode: host` is the load-bearing decision. Consequences:

- The container shares the host's network stack. No NAT, no separate IP.
- DDS multicast discovery works as if everything were running natively.
- Port 8080 / 9090 (UI server / rosbridge) bind directly to host interfaces.
- The autonomy stack outside the container and the camera / micro-ROS publishers inside the container see each other on the same DDS domain (`ROS_DOMAIN_ID=42`).

The race-day launch file comments explicitly call out that CycloneDDS must run with **stock multicast settings** on both sides; an attempt to pin the host autonomy stack to a unicast-only LAN profile bottlenecked the colour-image stream. Both sides use defaults now.

## Container interactions

```
┌──────────────────────────────────────────────────┐
│ Jetson host                                      │
│                                                  │
│ ┌────────────────────────────────────────────┐   │
│ │ Native autonomy stack                      │   │
│ │  race_day.launch.py                        │   │
│ │  → image_preprocessor, behavior_planner..  │   │
│ │  RMW=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=42  │   │
│ └────────────────────────────────────────────┘   │
│                  ▲ DDS multicast (host network)  │
│ ┌────────────────────────────────────────────┐   │
│ │ Container: bfmc_drivers (host network)     │   │
│ │  realsense2_camera                         │   │
│ │  micro_ros_agent on /dev/ttyACM0           │   │
│ │  RMW=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=42  │   │
│ └────────────────────────────────────────────┘   │
│                  │ USB │ UART                    │
└──────────────────│─────│─────────────────────────┘
                   ▼     ▼
              RealSense  STM32F401RE
              D435i      (firmware repo)
```

## Dead / unused container artefacts

None observed. The Dockerfile ships `ros-humble-can-msgs` + `can-utils` (CAN tooling) which the deployed launch does **not** exercise; this is a deliberate vehicle-platform-flexibility hedge, not dead code, but the public release documentation should note it.

## What this audit does not cover

- The runtime-validation `enable_runtime_validation` arg path is a host-side concern (launches `health_monitor_node`); not container-related.
- The image is not multi-arch; it is intended for ARM64 Jetson only. This is implicit, not declared in the Dockerfile.
- Image size, layer ordering, and cache friendliness are not profiled. The container is built once per machine; subsequent rebuilds are infrequent.
