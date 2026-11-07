# Race-day launch — explicit, conservative, P1–P5 defaults baked in.
#
# Brings up: drivers + fusion + perception + planning + control + safety + ui.
# Differs from hil_bringup mainly by:
#   * Pinning every P1–P4 tunable explicitly so a code-default drift cannot
#     silently re-introduce a regression.
#   * Using the EXPANDED mission + semantics YAMLs (BFS gap-filled, every
#     inserted node tagged) so the runtime cannot encounter an untagged node.
#   * Starting bfmc_ui_node (publishes /ui/status JSON for the dashboard).
#   * Health monitor / runtime validation off by default — turn on with the
#     `enable_runtime_validation:=true` launch arg.

import os

from launch import LaunchDescription
from launch.actions import (
    TimerAction, DeclareLaunchArgument, RegisterEventHandler, LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# Race-day DDS profile (CycloneDDS pinned to WiFi NIC, LAN-only). Installed
# to share/system_bringup/config/. Exporting CYCLONEDDS_URI here makes every
# subsequently-spawned node inherit it without operator intervention.
_dds_xml = os.path.join(
    get_package_share_directory('system_bringup'), 'config', 'cyclonedds_race.xml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # TensorRT engine for object_detection_node. Empty = use model.engine_path
    # from object_detection.yaml.
    engine_path = LaunchConfiguration('engine_path', default='')
    enable_runtime_validation = LaunchConfiguration(
        'enable_runtime_validation', default='false')

    graph_path = os.path.join(
        get_package_share_directory('global_path_planner_cpp'),
        'maps', 'Competition_track_graph.graphml')
    zones_path = os.path.join(
        get_package_share_directory('global_path_planner_cpp'),
        'maps', 'zones.yaml')
    mission_config_dir = os.path.join(
        get_package_share_directory('mission_supervisor_cpp'), 'config')

    # ----- DRIVERS ---------------------------------------------------------
    imu_driver = Node(
        package='imu_driver', executable='imu_driver_node',
        name='imu_driver_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    localization_client = Node(
        package='ekf_localization', executable='localization_client_node',
        name='localization_client_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'server_host': '', 'server_port': 0, 'frame_id': 'map',
            'reconnect_delay_s': 1.0, 'recv_timeout_s': 2.0,
        }])

    runtime_telemetry = Node(
        package='runtime_telemetry_cpp', executable='runtime_telemetry_node',
        name='runtime_telemetry_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- PERCEPTION ------------------------------------------------------
    # Camera = 15 Hz (set in docker/realsense_params.yaml). Lane is callback-
    # driven so it inherits. free_space and object_detection cap themselves
    # below via min_period_s.
    perception_container = ComposableNodeContainer(
        name='perception_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='image_preprocessor_cpp',
                plugin='image_preprocessor_cpp::ImagePreprocessorNode',
                name='image_preprocessor_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    # Race-day: 100 ms was too tight. One USB stutter or
                    # thermal hiccup triggered the whole pipeline to start
                    # dropping cascade-style (logged: 105 → 197 → 1243 →
                    # 42000 ms over 10 s). 500 ms tolerates real-world
                    # USB/RealSense jitter while still rejecting truly
                    # stale frames.
                    'stale_threshold_s': 0.50,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='lane_geometry_cpp',
                plugin='lane_geometry_cpp::LaneGeometryNode',
                name='lane_geometry_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'stale_threshold_s': 0.50,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='free_space_cpp',
                plugin='free_space_cpp::FreeSpaceNode',
                name='free_space_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_period_s':      0.10,
                    'stale_threshold_s': 0.50,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='screen', emulate_tty=True, respawn=True, respawn_delay=2.0)

    obj_det = Node(
        package='object_detection_cpp', executable='object_detection_node',
        name='object_detection_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':      use_sim_time,
            'min_period_s':      0.125,
            'stale_threshold_s': 0.50,
            'engine_path':       ParameterValue(engine_path, value_type=str),
        }])

    obj_tracker = Node(
        package='object_tracker_cpp', executable='object_tracker_node',
        name='object_tracker_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- PLANNING --------------------------------------------------------
    # Mission supervisor owns /planning/global_path and /navigation/flags.
    # Global path planner stays alive but `disable_pub=True` so it doesn't race.
    global_planner = Node(
        package='global_path_planner_cpp', executable='global_path_planner_node',
        name='global_path_planner_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':        use_sim_time,
            'graphml_path':        graph_path,
            'zones_yaml_path':     zones_path,
            'disable_pub':         True,
            'goal_node':           '261',
            'start_node':          '263',
            'algorithm':           'astar',
            'replan_deviation_m':  0.40,
            'republish_period_s':  1.0,
            # P1: replan storm guard.
            'replan_consecutive_samples': 5,
            'replan_min_speed_mps':       0.10,
            'replan_min_interval_s':      3.0,
        }])

    # Race-day: point at the BFS-expanded mission + per-node semantics (420
    # nodes, every node tagged, 0 validation errors). The base files
    # (competition_mission.yaml + competition_semantics.yaml) have 15 graph
    # gaps; the *_expanded.yaml siblings auto-filled them via BFS and the
    # inheritance pass tagged every inserted node.
    mission_supervisor = Node(
        package='mission_supervisor_cpp', executable='mission_semantics_node',
        name='mission_semantics_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':   use_sim_time,
            'supervisor_hz':  10.0,
            'graphml_path':   graph_path,
            'config_dir':     mission_config_dir,
            'mission_yaml':   mission_config_dir + '/competition_mission_expanded.yaml',
            'semantics_yaml': mission_config_dir + '/competition_semantics_expanded.yaml',
            # Hysteresis on nearest-node (already in S3): 0.3 m or 0.5 s.
            'hys_distance_margin_m': 0.30,
            'hys_persist_s':         0.50,
        }])

    v2x_bridge = Node(
        package='v2x_bridge_cpp', executable='v2x_bridge_node',
        name='v2x_bridge_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time, 'udp_port': 5007,
            'tcp_host': '', 'tcp_port': 9000, 'locsys_id': 0,
            'tx_rate_hz': 2.0, 'enable_tcp': False,
        }])

    path_planner = Node(
        package='local_path_planner_cpp', executable='local_path_planner_node',
        name='local_path_planner', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time, 'planner_hz': 20.0,
            'lookahead_m': 2.0, 'num_points': 40,
            'a_max': 1.8, 'a_lat_max': 0.8,
            'obstacle_radius': 0.30, 'obstacle_lookahead_max': 2.5,
            'lane_min_confidence': 0.20,
        }])

    traj_gen = Node(
        package='trajectory_generator', executable='trajectory_generator_node',
        name='trajectory_generator_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- SAFETY ----------------------------------------------------------
    safety_supervisor = Node(
        package='safety', executable='safety_supervisor_node',
        name='safety_supervisor_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_max':  0.35,
            'reverse_max':  0.0,    # P3: no reverse possible at the supervisor.
            'deadband':     0.05,
            'max_brake':    0.5,    # R4: brake ceiling well below 1.0 (firmware bug guard).
        }])

    # R1: emergency persistence — soft signals must stay bad for 5 ticks
    # (1.0 s at 5 Hz) before STOP latches. Hard signals (manual/watchdog/
    # vehicle_ready=false) stay instant.
    emergency_stop = Node(
        package='safety', executable='emergency_stop_node',
        name='emergency_stop_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'emergency_persist_ticks': 5,
        }])

    # P2: READY enter=20 valid, exit=50 invalid (10 s assert, 25 s drop).
    system_state_machine = Node(
        package='safety', executable='system_state_machine_node',
        name='system_state_machine_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'tick_period_s':       0.50,
            'ready_enter_samples': 20,
            'ready_exit_samples':  50,
        }])

    # Static TF tree (unchanged from hil_bringup).
    tf_static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_map_to_odom', output='screen', emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])
    tf_static_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_camera', output='screen', emulate_tty=True,
        arguments=['0.235', '0', '0.155', '0', '0', '0', 'base_link', 'camera_link'])
    tf_static_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_imu', output='screen', emulate_tty=True,
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'])

    # ----- FUSION (lifecycle) ----------------------------------------------
    ekf = LifecycleNode(
        package='ekf_localization', executable='ekf_localization_node',
        name='ekf_localization_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    # Race-day: lane_age 204 ms was just above the 200 ms cap, causing
    # drift_estimator to skip ticks every few seconds. 500 ms tolerates
    # the same USB/DDS jitter the perception chain now allows.
    drift_est = LifecycleNode(
        package='drift_estimator', executable='drift_estimator_node',
        name='drift_estimator_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_debug_topics': enable_runtime_validation,
            'stale_state_max_s': 0.50,
            'stale_lane_max_s':  0.50,
        }])
    veh_state = LifecycleNode(
        package='vehicle_state', executable='vehicle_state_node',
        name='vehicle_state_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    tf2_pub = LifecycleNode(
        package='tf2_publisher', executable='tf2_publisher_node',
        name='tf2_publisher_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # P4: behavior planner = 5 Hz tick.
    # R2: soft-degradation — transient low confidence → LANE_KEEP @ 0.20 m/s,
    # only EMERGENCY_STOP after `degraded_persist_ticks` (10 @ 5 Hz = 2.0 s).
    # F1: DetectionTimer engage windows tightened — at 8 Hz detector, 0.15 s
    # confirm = ~1.2 frames engage delay (was 0.25 s = ~2 frames). Clear
    # windows kept loose to tolerate flicker on disengage.
    behavior = LifecycleNode(
        package='behavior_planner_cpp', executable='behavior_planner_node',
        name='behavior_planner_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':           use_sim_time,
            'tick_period_s':          0.20,
            'enable_csv_logging':     False,
            'degraded_speed':         0.20,
            'degraded_persist_ticks': 10,
            # Bug-3: stiffer DetectionTimer hysteresis to reject 1-frame
            # YOLOv8 ghosts in static / cluttered backgrounds. At 8 Hz
            # detector, 0.40 s = ~3 frames confirm. Pedestrian stays
            # responsive (life-safety).
            'pedestrian_confirm_s':   0.15,
            'pedestrian_clear_s':     1.00,
            'vehicle_confirm_s':      0.40,
            'vehicle_clear_s':        0.80,
            'sign_confirm_s':         0.30,
            'sign_clear_s':           0.60,
            'roadblock_confirm_s':    0.40,
            'roadblock_clear_s':      1.50,
            # FSM #98 — race-day finals signs (no_entry / priority).
            'no_entry_dist_thresh':   1.50,
            'turn_speed_priority':    0.30,

            # S1 — finals speed table (competitive cruise, still safe):
            # stopping distance @ 0.30 m/s = 0.107 m (10× margin to nearest gate)
            # stopping distance @ 0.55 m/s = 0.255 m (4× margin to follow gate)
            'city_speed':             0.30,
            'highway_speed':          0.55,
            'turn_speed':             0.22,
            'roundabout_speed':       0.18,
            'crosswalk_speed':        0.12,
            # parking_speed 0.08 → 0.12 — at 0.08 the controller threw
            # ~0.063 throttle, mapping to PWM ≈ 1524 µs (only 39 µs above
            # neutral, inside most ESC deadbands). 0.12 maps to ~1545 µs
            # which the Quicrun engages reliably.
            'parking_speed':          0.12,
            'parking_dist_thresh':    0.80,
            'roundabout_dist_thresh': 1.50,
            'follow_slow_speed':      0.12,
            # Bug-2: tailing_min_speed lifted off the controller's stop
            # threshold. At 0.05 the EMA converged to exactly 0.05 and the
            # controller fell to neutral; 0.10 keeps a clear forward command.
            'tailing_min_speed':      0.10,

            # S2 — overtake parameters.
            'overtake_lateral_m':     0.40,   # one BFMC lane width
            'overtake_ramp_s':        1.00,   # sigmoid ramp 0→0.40
            'overtake_hold_s':        1.50,   # hold in left lane (passes a 35-cm car @ 0.30 m/s in ~1.2 s)
            'overtake_return_s':      1.00,   # sigmoid back 0.40→0
        }])

    # ----- CONTROL (lifecycle) ---------------------------------------------
    # P3: throttle ∈ [0, 0.35], brake ∈ [0, 0.5], goal-reached neutral latch.
    # P4: 20 Hz control loop.
    # R3: speed estimator uses /fusion/imu_confidence (subscribed in code).
    # R4: abs_max_brake tightened to 0.5 — matches supervisor's max_brake.
    traj_ctrl = LifecycleNode(
        package='bfmc_control', executable='trajectory_controller_node',
        name='trajectory_controller_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':     use_sim_time,
            'control_rate_hz':  20.0,
            'abs_max_throttle': 0.35,
            'abs_max_brake':    0.5,
            'goal_radius_m':    0.25,
            'goal_v_eps':       0.05,
            'max_brake':        0.45,
        }])

    # ----- UI --------------------------------------------------------------
    # /ui/status JSON aggregator @ 5 Hz (consumed by dashboard.html).
    ui = Node(
        package='bfmc_ui_cpp', executable='bfmc_ui_node',
        name='bfmc_ui_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'publish_hz': 5.0}])

    # rosbridge_server (ws://0.0.0.0:9090) — the legacy Bosch React UI talks
    # to this. Apt-installed: ros-humble-rosbridge-server.
    rosbridge = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090,
            'address': '0.0.0.0',
            'use_compression': False,
        }])

    # tf2_web_republisher — streams TF over rosbridge. Required by the legacy
    # React UI map view. Apt-installed: ros-humble-tf2-web-republisher.
    # Skip silently if the package isn't installed yet (launch still works,
    # the map view just won't draw TF frames; our dashboard.html draws the
    # car from /vehicle/state directly so it's unaffected).
    tf2_web = None
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('tf2_web_republisher')
        tf2_web = Node(
            package='tf2_web_republisher', executable='tf2_web_republisher',
            name='tf2_web_republisher', output='screen', emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    except Exception:
        # Don't print here — launch description construction shouldn't have
        # side effects. The absence will be obvious in `ros2 node list`.
        tf2_web = None

    # Static-file HTTP server on :8080 — serves React build + dashboard.html.
    ui_server = Node(
        package='bfmc_ui_server', executable='bfmc_ui_server_node',
        name='bfmc_ui_server_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'port': 8080}])

    # ----- ORCHESTRATION ---------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_autonomy', output='screen', emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'lane_geometry_node', 'free_space_node',
                'ekf_localization_node', 'drift_estimator_node',
                'vehicle_state_node', 'tf2_publisher_node',
                'behavior_planner_node', 'trajectory_controller_node',
            ]},
            {'bond_timeout': 0.0},
            {'attempt_respawn_reconnection': False},
        ])
    manager_trigger = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=traj_ctrl,
            on_start=[
                LogInfo(msg="[RACE-DAY] Lifecycles spawned. Manager in 2s..."),
                TimerAction(period=2.0, actions=[lifecycle_manager]),
            ]))

    watchdog = Node(
        package='safety', executable='watchdog_node',
        name='watchdog_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    watchdog_trigger = TimerAction(
        period=10.0,
        actions=[LogInfo(msg="[RACE-DAY] Activating Safety Watchdog..."), watchdog])

    health_monitor = Node(
        package='health_monitor_cpp', executable='health_monitor_node',
        name='health_monitor_node', output='screen', emulate_tty=True,
        condition=IfCondition(enable_runtime_validation),
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate_window_s': 5.0,
            'metrics_period_s': 1.0,
            'summary_period_s': 5.0,
        }])

    return LaunchDescription([
        # R7: pin CycloneDDS to LAN-only profile for every child process.
        # If the file is missing (workspace not installed yet), CycloneDDS
        # falls back to defaults — operator will see warnings, not crashes.
        # RACE-DAY FIX: do NOT force CYCLONEDDS_URI to the LAN-only profile.
        # The docker container running RealSense + microROS uses DEFAULT
        # CycloneDDS settings (full multicast OK). Forcing our spdp-only
        # XML on the autonomy host created a unicast-only data path that
        # bottlenecked the larger color frames (small depth survived).
        # Both sides now use stock CycloneDDS → wire-compatible, full speed.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('engine_path', default_value='',
            description='Absolute path to the TensorRT engine for object_detection_node. '
                        'Empty keeps model.engine_path from object_detection.yaml.'),
        DeclareLaunchArgument('enable_runtime_validation', default_value='false'),

        # Drivers
        imu_driver, localization_client, runtime_telemetry,
        # Perception
        perception_container, obj_det, obj_tracker,
        # Planning
        global_planner, mission_supervisor, v2x_bridge, path_planner, traj_gen,
        # Safety
        safety_supervisor, emergency_stop, system_state_machine,
        # TF
        tf_static_map_odom, tf_static_camera, tf_static_imu,
        # Fusion + planner/control lifecycles
        ekf, drift_est, veh_state, tf2_pub, behavior, traj_ctrl,
        # UI: status aggregator, rosbridge, tf2 web stream, static file server
        ui, rosbridge, ui_server,
        # Optional
        health_monitor,
        # Sequencers
        manager_trigger, watchdog_trigger,
    ] + ([tf2_web] if tf2_web else []))
