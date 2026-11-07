import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


# ----------------------------------------------------------------------------
# Phase 3.1: race-day rosbag recording list.
#
# This launch file does NOT auto-record a bag — the operator decides when and
# where. The variable below is the canonical "what we want in every track-day
# bag" list, kept here so the launch and the bag stay in sync.
#
# Suggested usage (run in a separate terminal):
#   ros2 bag record -o ~/bfmc_runs/bag-$(date +%F_%H%M%S) \
#       /perception/objects \
#       /perception/tracked_objects \
#       /perception/debug/track_count \
#       /perception/debug/active_tracks \
#       /perception/debug/tracker_latency_ms \
#       /perception/debug/id_switch_count
#
# Or programmatically:
#   ros2 bag record -o ~/bfmc_runs/bag-$(date +%F_%H%M%S) "${TRACKED_TOPICS[@]}"
# where TRACKED_TOPICS is the shell expansion of this same list.
# ----------------------------------------------------------------------------
TRACKED_TOPICS = [
    "/perception/objects",
    "/perception/tracked_objects",
    "/perception/debug/track_count",
    "/perception/debug/active_tracks",
    "/perception/debug/tracker_latency_ms",
    "/perception/debug/id_switch_count",
]



from launch.actions import TimerAction, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Master Orchestrator for BFMC PHYSICAL Autonomy Stack.
    Fully integrated Drivers, Perception, Fusion, Control, and Telemetry.

    Perception layout:
        - image_preprocessor_cpp + lane_geometry_cpp + free_space_cpp are
          loaded as composable nodes in a single multi-threaded container
          (perception_container) with use_intra_process_comms=True, so the
          preprocessed image is handed to lane_geometry by pointer (zero copy,
          zero serialization) instead of going through DDS.
        - object_detection_cpp stays a standalone process (TensorRT engine
          lifecycle + CUDA stream ownership left untouched per design freeze).
        - lifecycle_manager addresses lane_geometry_node and free_space_node
          by name; hosting them in a container does not change their service
          paths, so the manager keeps working unchanged.
    """

    # CRITICAL: Must be false for physical hardware!
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # TensorRT engine for object_detection_node. Empty = use model.engine_path
    # from object_detection.yaml.
    engine_path = LaunchConfiguration('engine_path', default='')

    # Optional runtime-validation layer. When 'true':
    #   • health_monitor_node starts and publishes /system/health/{status,metrics}
    #     and writes ~/bfmc_runs/run-<ts>/validation_summary.yaml every 5 s.
    #   • drift_estimator runs with enable_debug_topics=true so
    #     /fusion/debug/{lane_heading,curvature_weight,weighted_error,
    #     node_latency_ms,message_age_ms,correction_active} are available.
    # Default 'false' for race-day production — zero overhead path unchanged.
    enable_runtime_validation = LaunchConfiguration(
        'enable_runtime_validation', default='false')

    # Output directory for validation_summary.yaml. Only consulted by
    # health_monitor when enable_runtime_validation is true. ~ is expanded
    # below via os.path.expanduser so the C++ node never sees a tilde.
    validation_output_dir = LaunchConfiguration(
        'validation_output_dir',
        default=os.path.expanduser('~/bfmc_runs'))

    # ========================================================================
    # 1. NORMAL NODES (Start instantly, self-managing)
    # ========================================================================

    # -- Hardware Drivers --
    imu_driver = Node(
        package='imu_driver', executable='imu_driver_node',
        name='imu_driver_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -- Localization TCP-JSON client (BFMC race server) -----------------------
    # Publishes geometry_msgs/PointStamped on /localization/raw_position.
    # Defaults are empty: the node idles silently when no endpoint is set.
    # When deploying to the competition arena, set server_host + server_port
    # via a YAML/CLI override or this launch file. EKF runs on IMU-only
    # dead reckoning if no fixes arrive, so the vehicle is never frozen by
    # localization absence.
    localization_client = Node(
        package='ekf_localization', executable='localization_client_node',
        name='localization_client_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'server_host': '',
            'server_port': 0,
            'frame_id': 'map',
            'reconnect_delay_s': 1.0,
            'recv_timeout_s': 2.0,
        }]
    )

    # ultra_driver removed (Phase Final): no ultrasonic hardware on the
    # OdyssiX build, and safety_supervisor no longer subscribes to
    # /microros/ultrasonic_front. Leaving the driver running was a no-op but
    # consumed micro-ROS pub slots on the agent.

    # Phase Final P6: lightweight runtime telemetry (CPU, RAM, thermals) →
    # /system/runtime_health. 1 Hz, sysfs-only, no tegrastats subprocess.
    runtime_telemetry = Node(
        package='runtime_telemetry_cpp', executable='runtime_telemetry_node',
        name='runtime_telemetry_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -- Perception container (image_preprocessor + lane_geometry + free_space) --
    # All three share one process, one CPU-affinity domain, and intra-process
    # zero-copy transport for /perception/image_preprocessed.
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='image_preprocessor_cpp',
                plugin='image_preprocessor_cpp::ImagePreprocessorNode',
                name='image_preprocessor_node',
                parameters=[{'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lane_geometry_cpp',
                plugin='lane_geometry_cpp::LaneGeometryNode',
                name='lane_geometry_node',
                parameters=[{'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='free_space_cpp',
                plugin='free_space_cpp::FreeSpaceNode',
                name='free_space_node',
                parameters=[{'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    # C++ port: ultra-low-latency TensorRT 10 YOLOv8 detector.
    # Kept as a standalone process (CUDA / TRT owns its own context).
    obj_det = Node(
        package='object_detection_cpp', executable='object_detection_node',
        name='object_detection_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'engine_path':  ParameterValue(engine_path, value_type=str),
        }]
    )

    # ByteTrack adapted for 3D vehicle-frame positions. Subscribes
    # /perception/objects (detector raw), publishes /perception/tracked_objects
    # with the same message type but the DetectedObject.id field now carrying
    # a STABLE uint8 track id and DetectedObject.velocity populated by the KF.
    # CPU-only; no CUDA dependency; standalone process.
    obj_tracker = Node(
        package='object_tracker_cpp', executable='object_tracker_node',
        name='object_tracker_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Phase Final: global path planner (graphml + zones.yaml side-car).
    # Subscribes /vehicle/state, publishes /planning/global_path (nav_msgs/Path)
    # and /navigation/flags (String) for behavior_planner_cpp.
    # graphml_path / zones_yaml_path default to the repo-bundled files in
    # share/global_path_planner_cpp/maps/. Override via launch args at the
    # track ('goal_node' is the next mandatory override).
    global_planner = Node(
        package='global_path_planner_cpp', executable='global_path_planner_node',
        name='global_path_planner_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':   use_sim_time,
            'graphml_path':   os.path.join(get_package_share_directory('global_path_planner_cpp'), 'maps', 'Competition_track_graph.graphml'),
            'zones_yaml_path':os.path.join(get_package_share_directory('global_path_planner_cpp'), 'maps', 'zones.yaml'),
            # Mission supervisor is the authoritative source for the mission
            # path on /planning/global_path and the region tag on
            # /navigation/flags. We keep this planner alive (for /vehicle/state
            # consumption, nearest-node debug topic, etc.) but suppress its
            # publishes so the two providers don't race.
            'disable_pub':     True,
            'goal_node':       '261',
            'start_node':      '263',
            'algorithm':      'astar',
            'replan_deviation_m': 0.40,
            'republish_period_s': 1.0,
        }]
    )

    # Phase Final — mission supervisor (central semantic provider).
    # Loads 5 YAMLs + GraphML, validates the mission against the graph at
    # startup, publishes /planning/global_path (authoritative mission path),
    # /planning/competition_path (visualization duplicate), /navigation/flags
    # (region tag for behavior_planner), and the live telemetry topics
    # /planning/current_node/region/behavior/speed_limit/mission_progress/
    # localization_confidence/mission_score_estimate. RViz marker array on
    # /planning/competition_markers.
    mission_supervisor = Node(
        package='mission_supervisor_cpp', executable='mission_semantics_node',
        name='mission_semantics_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':   use_sim_time,
            'supervisor_hz':  10.0,
            'graphml_path':   os.path.join(get_package_share_directory('global_path_planner_cpp'), 'maps', 'Competition_track_graph.graphml'),
            'config_dir':     os.path.join(get_package_share_directory('mission_supervisor_cpp'), 'config'),
        }]
    )

    # Phase Final: V2X bridge (UDP:5007 receive + TCP location uplink).
    # tcp_host empty by default ⇒ uplink loop idles. Set at the track via
    # `--ros-args -p tcp_host:=<server-ip> -p locsys_id:=<id> -p enable_tcp:=true`.
    v2x_bridge = Node(
        package='v2x_bridge_cpp', executable='v2x_bridge_node',
        name='v2x_bridge_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'udp_port':     5007,
            'tcp_host':     '',
            'tcp_port':     9000,
            'locsys_id':    0,
            'tx_rate_hz':   2.0,
            'enable_tcp':   False,
        }]
    )

    # Phase B: swap Python local_path_planner → C++ local_path_planner_cpp.
    # Same publish topic (`/planning/local_path` String JSON) and node name
    # so trajectory_generator + downstream consumers stay unchanged. Adds
    # /planning/path_confidence (Float64) and obstacle clipping from
    # /perception/tracked_objects.
    path_planner = Node(
        package='local_path_planner_cpp', executable='local_path_planner_node',
        name='local_path_planner', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':         use_sim_time,
            'planner_hz':           20.0,
            'lookahead_m':          2.0,
            'num_points':           40,
            'a_max':                1.8,
            'a_lat_max':            0.8,
            'obstacle_radius':      0.30,
            'obstacle_lookahead_max': 2.5,
            'lane_min_confidence':  0.20,
        }]
    )

    traj_gen = Node(
        package='trajectory_generator', executable='trajectory_generator_node',
        name='trajectory_generator_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -- Safety --
    safety_supervisor = Node(
        package='safety', executable='safety_supervisor_node',
        name='safety_supervisor_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Phase 7: emergency_stop_node — OR aggregator across imu_conf / state
    # confidence / state stale / vehicle_ready / manual / watchdog. Publishes
    # /safety/emergency (for behavior_planner) AND /safety/emergency_stop
    # (same topic the watchdog uses, multiple publishers permitted).
    emergency_stop = Node(
        package='safety', executable='emergency_stop_node',
        name='emergency_stop_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Phase B: system_state_machine_node — gates /system/vehicle_ready until
    # IMU + fusion + localization + perception + planning + control are all
    # alive. When NOT ready, emergency_stop_node sees /system/vehicle_ready=false
    # and asserts emergency → behavior FSM goes EMERGENCY_STOP → controller
    # publishes neutral. No motion possible until READY_TO_DRIVE.
    system_state_machine = Node(
        package='safety', executable='system_state_machine_node',
        name='system_state_machine_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Phase 8: static TF publishers (single broadcaster per edge).
    # map → odom (identity for now; once a global localization comes online
    # this is the place to inject its correction). All values are in metres /
    # radians: x y z qx qy qz qw parent_frame child_frame.
    tf_static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_map_to_odom', output='screen', emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    # base_link → camera_link: camera mounted ~+0.235 m forward,
    # ~+0.155 m above chassis, tilted ~2° forward (already compensated by
    # STM32; we publish the geometric offset only).
    tf_static_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_camera', output='screen', emulate_tty=True,
        arguments=['0.235', '0', '0.155', '0', '0', '0', 'base_link', 'camera_link']
    )
    # base_link → imu_link: BNO085 sits on the STM32 board near the chassis
    # centre; small offset only.
    tf_static_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_imu', output='screen', emulate_tty=True,
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    # COMMENTED OUT FOR NOW
    # rosbridge = Node(
    #     package='rosbridge_server', executable='rosbridge_websocket',
    #     name='rosbridge_websocket', output='screen', emulate_tty=True,
    #     parameters=[{'port': 9090, 'use_sim_time': use_sim_time}]
    # )

    # ========================================================================
    # 2. LIFECYCLE NODES (Boot UNCONFIGURED, wait for Manager)
    # ========================================================================
    ekf = LifecycleNode(
        package='ekf_localization', executable='ekf_localization_node',
        name='ekf_localization_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    drift_est = LifecycleNode(
        package='drift_estimator', executable='drift_estimator_node',
        name='drift_estimator_node', namespace='', output='screen', emulate_tty=True,
        # enable_debug_topics is bound to enable_runtime_validation so the
        # health monitor's /fusion/debug/lane_heading subscriber gets data.
        # When validation is off (default), debug topics stay off — no overhead.
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_debug_topics': enable_runtime_validation,
        }]
    )

    veh_state = LifecycleNode(
        package='vehicle_state', executable='vehicle_state_node',
        name='vehicle_state_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf2_pub = LifecycleNode(
        package='tf2_publisher', executable='tf2_publisher_node',
        name='tf2_publisher_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Phase Final: swap Python behavior_planner → C++ behavior_planner_cpp.
    # Node name kept identical so lifecycle_manager (below) controls it by
    # name without any change to its node_names list.
    behavior = LifecycleNode(
        package='behavior_planner_cpp', executable='behavior_planner_node',
        name='behavior_planner_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            # CSV logging off by default — flip on for track-day post-mortems.
            'enable_csv_logging': False,
            'csv_log_dir': '/var/log/bfmc/planning',
        }]
    )

    # FIXED PACKAGE NAME TO bfmc_control
    traj_ctrl = LifecycleNode(
        package='bfmc_control', executable='trajectory_controller_node',
        name='trajectory_controller_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========================================================================
    # 3. ORCHESTRATION (The Magic Timing)
    # ========================================================================
    # Lifecycle manager still drives lane_geometry_node and free_space_node by
    # name - they're inside the container but their service paths are unchanged.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_autonomy', output='screen', emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'lane_geometry_node', 'free_space_node',
                'ekf_localization_node', 'drift_estimator_node', 'vehicle_state_node', 'tf2_publisher_node',
                'behavior_planner_node', 'trajectory_controller_node'
            ]},
            {'bond_timeout': 0.0},
            {'attempt_respawn_reconnection': False}
        ]
    )

    # Launch Manager 2 seconds after the final lifecycle node spawns
    manager_trigger = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=traj_ctrl,
            on_start=[
                LogInfo(msg="[ORCHESTRATOR] Lifecycles spawned. Starting manager in 2s..."),
                TimerAction(period=2.0, actions=[lifecycle_manager])
            ]
        )
    )

    watchdog = Node(
        package='safety', executable='watchdog_node',
        name='watchdog_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -- Runtime validation layer (optional) ----------------------------------
    # Subscribes to vehicle/state, tf, lane_geometry, yaw_bias_correction,
    # lane_heading (debug), imu_raw, color/image_raw. Publishes
    # /system/health/status and /system/health/metrics. Writes
    # validation_summary.yaml every 5 s into the per-launch run dir.
    health_monitor = Node(
        package='health_monitor_cpp', executable='health_monitor_node',
        name='health_monitor_node', output='screen', emulate_tty=True,
        condition=IfCondition(enable_runtime_validation),
        parameters=[{
            'use_sim_time': use_sim_time,
            'validation_output_dir': validation_output_dir,
            'rate_window_s': 5.0,
            'metrics_period_s': 1.0,
            'summary_period_s': 5.0,
        }]
    )

    # Delay watchdog by 10 seconds to allow the massive lifecycle chain to fully transition to ACTIVE
    watchdog_trigger = TimerAction(
        period=10.0,
        actions=[LogInfo(msg="[ORCHESTRATOR] Activating Safety Watchdog..."), watchdog]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('engine_path', default_value='',
            description='Absolute path to the TensorRT engine for object_detection_node. '
                        'Empty keeps model.engine_path from object_detection.yaml.'),
        DeclareLaunchArgument('enable_runtime_validation', default_value='false',
            description='Start health_monitor_node and enable drift_estimator debug topics.'),
        DeclareLaunchArgument('validation_output_dir',
            default_value=os.path.expanduser('~/bfmc_runs'),
            description='Directory in which run-<ts>/validation_summary.yaml is written.'),

        # Drivers (ultra_driver removed — no ultrasonic on this build)
        imu_driver, localization_client,
        runtime_telemetry,

        # Perception container (replaces standalone img_prep + lane_geom + free_space)
        perception_container,

        # Normal Nodes (removed rosbridge)
        obj_det, obj_tracker, global_planner, mission_supervisor, v2x_bridge,
        path_planner, traj_gen, safety_supervisor,
        # Phase 7 + Phase B + Phase 8
        emergency_stop, system_state_machine,
        tf_static_map_odom, tf_static_camera, tf_static_imu,

        # Lifecycle Nodes (lane_geom + free_space now live inside perception_container)
        ekf, drift_est, veh_state, tf2_pub, behavior, traj_ctrl,

        # Optional runtime validation (off by default)
        health_monitor,

        # Sequencers
        manager_trigger, watchdog_trigger
    ])
