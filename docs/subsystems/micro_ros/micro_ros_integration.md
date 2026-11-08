# micro-ROS Integration

Generated 2026-06-08. Scope: how the OdyssiX BFMC vehicle firmware (STM32F401RE) talks to the OdyssiX Brain (Jetson) over UART using micro-ROS / XRCE-DDS.

## End-to-end path

```
STM32 firmware (BoschFutureMobilityChallenge2026-Embedded)
   │
   │  micro-ROS client (rclc + rmw_microxrcedds)
   │  USART2 @ 460800 baud
   │
   ▼
USB (FTDI / ST-LINK virtual COM) → /dev/ttyACM0 (Jetson)
   │
   ▼
Docker container `bfmc_drivers`
   │
   │  micro_ros_agent (XRCE-DDS over serial)
   │
   ▼
DDS (CycloneDDS, ROS_DOMAIN_ID=42, multicast on host network)
   │
   ▼
Native autonomy stack (race_day.launch.py)
```

## Transport

| Property | Value | Source |
|---|---|---|
| Physical layer | USART2 over USB virtual COM | docker-compose `--dev /dev/ttyACM0` |
| Baud rate | 460 800 | docker-compose `-b 460800` |
| Framing | XRCE-DDS serial framing (default) | micro-ROS agent serial transport |
| Reliability | Reliable QoS (default rmw_microxrcedds) | upstream micro-ROS |
| Reconnect | Handled by micro-ROS session-sync logic + agent | upstream |
| Discovery | Pre-registered entities (publishers + subscribers) on session handshake | upstream |

The 460 800 baud rate is matched on both sides:

- Embedded: `Core/Src/usart.c` (firmware repo) sets USART2 to 460800.
- Host: `docker/docker-compose.yml` passes `-b 460800` to the agent.

## Topics across the bridge

The STM32 firmware publishes IMU data; the Brain subscribes via the agent. The Brain publishes control commands; the STM32 firmware subscribes via the agent.

### STM32 → Brain (sensor stream)

| STM32 publishes | Brain subscriber |
|---|---|
| `/microros/imu_raw` | `imu_driver_node` (`imu_driver_node.hpp:40`) |

The docker-compose agent command also passes `--remap` arguments to `_hardware`-suffixed names, but the deployed subscriber uses the unsuffixed `/microros/imu_raw`, as does `tools/check_run.sh`. See `docs/subsystems/docker/docker_audit.md`.

The firmware has no ultrasonic publisher. `ultrasonic_front_driver` (built but not deployed) subscribes to `/microros/ultrasonic_front`, which nothing currently publishes.

### Brain → STM32 (actuator command)

| Brain publishes | STM32 subscriber |
|---|---|
| `/control/command` (bfmc_msgs/ControlCommand) | `control_cmd_callback` in firmware `freertos.c` |

Throughput:

- IMU: 100 Hz batched packets from the BNO085, published by firmware as `Imu` messages → received by Brain at the agent's serial throughput limit (well under 100 Hz worth at 460 800 baud given 80–120 byte XRCE messages).
- Ultrasonic: not currently in race-day path (no subscriber active in race-day launch).
- Control command: 20 Hz from `trajectory_controller_node` → published through supervisor → reaches firmware via the agent.

## QoS

Both ends use micro-ROS defaults. The micro-ROS RMW maps to:

- Reliable per-message confirmation over the serial transport.
- Best-effort delivery if the session is broken (with reconnect retry).
- Single-publisher / single-subscriber model per topic (typical for one vehicle).

The Brain side sees these topics on the DDS bus with QoS profiles matching the agent's RTPS mapping (Reliable, KeepLast, history depth 10 is the default).

## Agent startup (deployed)

From `docker/docker-compose.yml`:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyACM0 \
    -b 460800 \
    --ros-args \
    --remap /microros/imu_raw:=/microros/imu_raw_hardware \
    --remap /microros/ultrasonic_front:=/microros/ultrasonic_front_hardware
```

A `while` loop polls for `/dev/ttyACM0` before launching the agent. The container exits if the agent exits — no automatic restart inside the container; the operator can re-`docker compose up` to recover.

## Session lifecycle (observed from firmware design)

| Phase | Behavior |
|---|---|
| Boot | STM32 firmware initialises peripherals, starts FreeRTOS, the `microROSTask` brings up the BNO085, configures the micro-ROS transport, allocator, entities. |
| Session establish | Brain-side agent waits on UART; STM32 client connects, registers publishers / subscribers. |
| Steady state | STM32 main loop runs at ~1 kHz (`osDelay(1)`). It publishes IMU at the BNO085 report rate, 100 Hz (10 000 µs report interval), reads UART RX ring for inbound control commands, writes PWM. `rmw_uros_sync_session` every ~1 s (1000 loop iterations) for time discipline. |
| Loss of agent | Brain side: agent process dies → DDS topic disappears. STM32 side: session lost → next `rclc_executor_spin_some` recovers via auto-recovery on 50 errors, then re-establishes. |
| Loss of STM32 | Brain side: agent loses session; on Jetson, `imu_driver_node` stops receiving on `/microros/imu_raw`. EKF starts emitting stale data. After 0.5 s (`stale_state_max_s`) drift estimator skips ticks. After more time, the system state machine drops READY. |
| Deadman | STM32 firmware applies a 500 ms command-silence deadman: if no `/control/command` reaches the STM32 in 500 ms, the firmware writes neutral PWM (1500 µs) until commands resume. |

## Failure modes

| Failure | Symptom on Brain | Mitigation |
|---|---|---|
| USB cable unplugged | `/dev/ttyACM0` disappears, agent exits, container exits. | Operator replugs; `docker compose up` again. |
| STM32 reset (fault, watchdog, boot button) | Agent loses session for a few seconds, then re-establishes after STM32 reboots and reconnects. | Firmware applies neutral PWM at boot, so the vehicle goes to safe state during the gap. |
| Baud mismatch | Garbage on UART; agent reports framing errors. | Pinned to 460 800 on both sides; no need to negotiate. |
| Multiple agents on the same `/dev/ttyACM0` | Only one process can hold the serial device. Second instance fails to open. | One container per vehicle. |
| Loss of DDS discovery | Container side: STM32 packets still arrive at agent. Host side: `imu_driver_node` doesn't see the topic. | Both sides run with `ROS_DOMAIN_ID=42`, CycloneDDS multicast on host network — usually robust. |
| Backpressure on serial | XRCE frame loss; micro-ROS reliability resends. | 460 800 is well above the steady-state message rate (~1 KB/s aggregate). Burst-tolerant by design. |

## Why these choices

- **Serial transport vs UDP** — chosen because the STM32 has no Ethernet stack and the BNO085 + control-loop work is already saturating one task. UART is the simplest reliable wire.
- **460 800 baud vs higher** — the STM32 UART driver runs interrupt-driven RX into a 2 KiB ring buffer. 460 800 is the highest standard rate that doesn't risk RX overruns under FreeRTOS scheduling. Higher rates (921 600, 1 Mbit) would need DMA RX (currently unused; documented in the embedded repo's audit).
- **Agent built from source** — the upstream apt `ros-humble-micro-ros-agent` package is unreliable / unavailable for ARM64 Jetson at this writing. Building from source against the same Humble release used by the autonomy stack avoids version drift.

## What this document does not claim

- Wire-protocol throughput is **not measured** in this repo. The 460 800 baud rate is sufficient by design but no benchmark trace is included.
- Session-recovery time after STM32 reset is **not measured** — it is whatever `rmw_microxrcedds` + the agent negotiate.
- The ultrasonic path is not live. The current firmware has no `/microros/ultrasonic_front` publisher (its only publisher is `/microros/imu_raw`), and the deployed race-day launch does not run `ultrasonic_front_driver`.
