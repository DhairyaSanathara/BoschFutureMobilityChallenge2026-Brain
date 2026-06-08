# Trade-offs

Generated 2026-06-08. Trade-off summaries for the major architectural choices. Each one is the negative space of a design decision in `docs/design_decisions/decisions.md`.

## 1. ROS 2 vs custom middleware

| | ROS 2 | Custom (ZMQ / DDS-direct / etc.) |
|---|---|---|
| Discovery | Multicast DDS built-in | Build it |
| Lifecycle | nav2 `lifecycle_manager` off-the-shelf | Build it |
| Tooling (rqt, rviz, bag) | Mature | None or DIY |
| QoS profiles | Reliable / Best-Effort / TransientLocal | Hand-roll |
| Composable / intra-process | First-class | DIY thread + shared queue |
| Learning curve | Concepts to onboard | Lower if team only knows one paradigm |
| Code velocity | High (ros2 pkg create, ros2 launch, ros2 topic) | Low |
| Race-day failure surface | Wide but mostly known | Narrow but unproven |

**Outcome.** ROS 2. The cost of building the tooling we'd need (lifecycle, composable nodes, QoS, micro-ROS pairing) far exceeds the cost of learning ROS 2.

## 2. micro-ROS vs direct serial protocol

| | micro-ROS | Direct serial (custom framing) |
|---|---|---|
| Maintenance | rmw_microxrcedds upstream | Owned forever |
| Reliability | Reliable QoS by default | DIY checksum + retry |
| Topic semantics | Identical to ROS 2 topics | Custom binary |
| Tooling | `ros2 topic echo /microros/imu_raw` works | None |
| Binary size on STM32F401 | Fits in 512 KiB FLASH with headroom | Smaller |
| Memory on STM32F401 | Uses 32 KiB custom-heap pool | Smaller |
| Latency | XRCE-DDS framing overhead | Lower bound |
| Debuggability | Standard tooling | Hand-debugged |

**Outcome.** micro-ROS. The STM32F401 has enough FLASH and RAM. Tooling-equivalence between embedded and Jetson sides is worth more than the latency / binary-size deltas.

## 3. Docker (driver-only) vs everything-native vs everything-containerised

| | All-native | Driver container (chosen) | All-container |
|---|---|---|---|
| Reproducibility of camera + agent | Manual | Built into image | Built into image |
| TensorRT / CUDA in autonomy | Native — uses Jetson stack directly | Same | Container needs CUDA / nvidia-runtime |
| Image size | n/a | Small (single layer for drivers) | Large |
| DDS multicast | Direct | `network_mode: host` makes it direct | `network_mode: host` makes it direct |
| Operator complexity | One terminal | Two layers (compose + launch) | One layer but big image build |
| Update agility | Native APT churn affects everything | Driver image can pin RealSense + agent versions | Same |

**Outcome.** Driver container only. The autonomy stack benefits from native CUDA; the drivers benefit from being pinned in an image.

## 4. Monolithic perception node vs N processes vs composable container

| | Monolithic | N processes | Composable container (chosen) |
|---|---|---|---|
| Topic-hop cost | None (in-function) | Full serialise + DDS | Zero-copy intra-process (when enabled) |
| Lifecycle | Single bag of state | Per-node | Per-node |
| Debug isolation | Hard | Easy | Easy |
| Failure isolation | One crash kills everything | Per-process | Per-node (but shares container process) |
| Build complexity | Big binary | One target per package | One target per package + plugin export |

**Outcome.** Composable container. The intra-process zero-copy benefit is significant on the image stream; per-node lifecycle is preserved.

## 5. nav2 lifecycle manager vs custom orchestrator vs no manager

| | Custom | None | nav2 manager (chosen) |
|---|---|---|---|
| Code we own | Lots | None | None |
| Battle-tested | No | n/a | Yes (years of nav2 use) |
| Auto-respawn behavior | DIY | n/a | Configurable |
| Bond protocol | DIY | n/a | Built-in |
| Deferred activation | DIY | n/a | Native (we use `TimerAction` to defer the manager itself) |

**Outcome.** nav2 lifecycle manager. The custom orchestrator path is a tar pit; "no manager" wastes the lifecycle infrastructure already in the nodes.

## 6. ROS 2 services / actions vs topic-only

| | Services + actions | Topics only (chosen) |
|---|---|---|
| Discoverability | Better for one-shot calls | Worse for one-shot |
| QoS | Per-method | Per-topic |
| State synchronisation | Synchronous | Eventual consistency |
| Race conditions | Localised to the call | Pervasive but tractable with TransientLocal |
| Lifecycle interaction | Services exist as lifecycle entities | Topics are loosely coupled |
| Race-day need | None observed | All our flows are streaming |

**Outcome.** Topics only. We had no use case that needed a one-shot request/response or a long-running action.

## 7. Pinning launch parameters vs YAML parameter files vs code defaults

| | Code defaults | YAML files | Launch-file pinning (chosen) |
|---|---|---|---|
| Single source of truth | Source code | YAML | Launch file |
| Drift risk | High (silent on any PR) | Medium (separate file forgotten) | Low (one screen) |
| Diff visibility on PR | Hidden in many files | One file | One file |
| Per-launch divergence | Hard | Multiple files | Easy (each launch pins its own values) |
| Operator review | Hard | Medium | Easy |

**Outcome.** Pin in the launch file. The launch file is the deployment artefact; any race-day-only deviation lives next to the node declaration.

## 8. C++ vs Python for deployed nodes

| | Python | C++ (chosen) |
|---|---|---|
| Iteration speed | Fast | Slow |
| Steady-state throughput | Bound by GIL + Python overhead | Bound by CPU |
| TensorRT / CUDA binding | Possible but awkward | First-class |
| 20 Hz behavior planner | Possible | Comfortable |
| 20 Hz controller | Marginal | Comfortable |
| Camera pipeline | Painful | Comfortable |

**Outcome.** Python for prototypes; C++ for deployed. The Python prototypes (image_preprocessor, behavior_planner, local_path_planner) remain in the repo for design provenance.

## 9. Mission supervisor owning path vs global planner owning path

| | Global planner owns | Mission supervisor owns (chosen) |
|---|---|---|
| Replanning model | On-demand A* | Fixed sequence with optional replan |
| Mission semantics tagging | Separate node | Co-located |
| Race-day robustness | Replan failure could blank the path | Path is always the BFS-expanded YAML |
| Debug visibility | Two nodes to inspect | One |
| Off-track recovery | Replan-driven | Future feature (currently relies on the fixed sequence) |

**Outcome.** Mission supervisor owns. Race-day mission is a known sequence; A* replan was a bigger surface than benefit.

## 10. Static map → odom transform vs dynamic localization client

| | Dynamic client | Static map=odom (chosen) |
|---|---|---|
| External localization dependency | Required | None |
| Race-day robustness | Depends on Bosch server | Self-contained |
| Localization-confidence input to fusion | Available | Only via lane-anchored drift estimator |
| Lap-start alignment | Server-corrected | Manual physical start |

**Outcome.** Static map=odom. The localization client is launched but configured with `server_host=''` → effectively off. The drift estimator carries the lane-anchored yaw correction; the EKF carries IMU-only odometry. This matches the race-day operating point.

## 11. Apache-2.0 vs MIT vs BSD-3-Clause for first-party

| | Apache-2.0 | MIT | BSD-3-Clause |
|---|---|---|---|
| Patent grant | Yes | No | No |
| Permissive | Yes | Yes | Yes |
| ROS 2 ecosystem norm | Common | Common | Common |
| Compatibility with bfmc_ui (BSD-3-Clause) | Yes | Yes | Yes |
| Compatibility with FreeRTOS (MIT) | Yes | Yes | Yes |
| Consistency with embedded release | Already Apache-2.0 | — | — |

**Outcome.** Apache-2.0 across first-party. Vendor licenses are preserved verbatim.
