# Security and Safety

This document is a guide for *safety-relevant* and *security-relevant* issues in the OdyssiX BFMC 2026 Brain. The Brain commands a moving vehicle. The safety boundary is split between the Jetson autonomy stack documented here and the STM32 firmware documented in the companion `BoschFutureMobilityChallenge2026-Embedded` repository.

## Safety boundary

The deployed system has three layered safeguards:

1. **Brain-side safety supervisor.** `safety_supervisor_node` clamps `/control/command_raw` to `forward_max=0.35`, `reverse_max=0.0`, `deadband=0.05`, `max_brake=0.5` before publishing `/control/command`. The lower clamp on reverse means the supervisor will not pass a negative throttle command at race-day settings.
2. **Brain-side emergency aggregator.** `emergency_stop_node` debounces soft signals over 5 ticks (1 s at 5 Hz) and immediately gates on hard signals (operator stop, watchdog alarm, `vehicle_ready=false`). `system_state_machine_node` requires 20 valid samples (10 s) before transitioning to READY and 50 invalid samples (25 s) before dropping it.
3. **Firmware-side command-silence deadman.** The STM32 firmware returns the steering servo and ESC to neutral PWM (1500 µs) within 500 ms of the last `/control/command` arriving. This is the authoritative safety guarantee — the only one that does not depend on the Brain process being healthy.

If a future change weakens any of these three, document the change in the relevant design-decision note and call out the impact on the safety boundary.

## Known safety gaps

These items are tracked release-engineering improvements. They do not currently affect the deployed safety boundary because the deployed launch wraps around them.

- **Drivers container runs privileged.** `docker-compose.yml` uses `privileged: true` to allow `/dev/ttyACM0` and the RealSense USB device access. Tightening to `--device` + cgroup rules is a possible follow-up; not relevant at race day where the operator owns the host.

- **CycloneDDS uses stock multicast.** The race-day launch file deliberately does **not** apply the LAN-only CycloneDDS profile (`system_bringup/config/cyclonedds_race.xml`) because it bottlenecked the color-image stream in earlier tests. Stock multicast is correct for the race-day LAN environment but would be inappropriate on a shared corporate network.

- **No image redaction.** The system publishes raw RGB camera frames on `/camera/camera/color/image_raw`. Anyone on the same DDS domain (`ROS_DOMAIN_ID=42`) can subscribe. The race-day environment is a closed LAN; in other deployments, consider DDS partitioning or topic-level access control.

## Vehicle operating envelope (race-day)

The deployed parameters cap vehicle behavior as follows:

| Limit | Source | Value |
|---|---|---|
| Maximum throttle | `trajectory_controller_node` + `safety_supervisor_node` | 0.35 |
| Maximum brake | `trajectory_controller_node` + `safety_supervisor_node` | 0.5 |
| Reverse throttle | `safety_supervisor_node` | 0.0 (disabled) |
| Throttle deadband | `safety_supervisor_node` | 0.05 |
| Control loop rate | `trajectory_controller_node` | 20 Hz |
| Behavior planner tick | `behavior_planner_node` | 5 Hz |
| Mission supervisor tick | `mission_semantics_node` | 10 Hz |
| Speed table — city | `behavior_planner_node` | 0.30 m/s |
| Speed table — highway | `behavior_planner_node` | 0.55 m/s |
| Speed table — crosswalk | `behavior_planner_node` | 0.12 m/s |
| Speed table — parking | `behavior_planner_node` | 0.12 m/s |
| Firmware command-silence deadman | STM32 (Embedded repo) | 500 ms → neutral PWM |

## Reporting

For safety-relevant issues (anything that could enable the vehicle to take a non-commanded action or hold actuator state past the 500 ms deadman):

- Open a *private* issue if your repository host supports it. Otherwise, contact the maintainers directly via the email in the package.xml maintainer field rather than filing publicly.
- Provide the launch file used, parameter values, and ROS 2 / JetPack / firmware versions.
- Include a reproduction if possible. Recorded rosbags are particularly useful.

Other issues (build, packaging, documentation, dependency updates) can go to the public issue tracker.

## Threat model out of scope

This project does not include:

- Cryptographic authentication of `/control/command` on the UART bus. The micro-ROS link is trusted; in deployment the only physical access to the UART is between the Brain container and the on-vehicle STM32 inside the chassis.
- Operator authentication for the rosbridge web socket. The race-day LAN is private; the rosbridge port (9090) is reachable to anyone on the LAN.
- Resistance to deliberate sensor spoofing (camera or IMU).
- Resistance to ROS 2 DDS-level intruders. ROS_DOMAIN_ID partitioning is not a security boundary.

These are deliberate scope decisions for a competition robotic vehicle running on a closed LAN.
