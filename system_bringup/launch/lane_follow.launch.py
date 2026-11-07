# Lane-follow-only minimal launch.
#
# Brings up: drivers + perception + fusion + behavior + control + safety + UI.
# DROPS: global_path_planner, mission_supervisor, local_path_planner_cpp (the
# mission/path nodes), v2x_bridge, health_monitor.
#
# Rationale: in lane-follow operation the vehicle doesn't need a graph mission
# or V2X. The behavior planner's LANE_KEEP path runs entirely from
# /perception/lane_geometry + /vehicle/state, and the trajectory_generator
# falls back to lane-only when /planning/global_path is silent. The behavior
# planner's emergency / FOLLOW triggers still work (tracker + detector chain
# is unaffected).
#
# Saves ~3 nodes worth of CPU + RAM and removes the "no global path" warning
# noise from the log.

import os

from launch import LaunchDescription
from launch.actions import (
    TimerAction, DeclareLaunchArgument, RegisterEventHandler, LogInfo,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


_dds_xml = os.path.join(
    get_package_share_directory('system_bringup'), 'config', 'cyclonedds_race.xml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # TensorRT engine for object_detection_node. Empty = use model.engine_path
    # from object_detection.yaml.
    engine_path = LaunchConfiguration('engine_path', default='')

    # ----- DRIVERS ---------------------------------------------------------
    imu_driver = Node(
        package='imu_driver', executable='imu_driver_node',
        name='imu_driver_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    runtime_telemetry = Node(
        package='runtime_telemetry_cpp', executable='runtime_telemetry_node',
        name='runtime_telemetry_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- PERCEPTION ------------------------------------------------------
    perception_container = ComposableNodeContainer(
        name='perception_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='image_preprocessor_cpp',
                plugin='image_preprocessor_cpp::ImagePreprocessorNode',
                name='image_preprocessor_node',
                parameters=[{'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='lane_geometry_cpp',
                plugin='lane_geometry_cpp::LaneGeometryNode',
                name='lane_geometry_node',
                parameters=[{'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='free_space_cpp',
                plugin='free_space_cpp::FreeSpaceNode',
                name='free_space_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_period_s': 0.10,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='screen', emulate_tty=True, respawn=True, respawn_delay=2.0)

    obj_det = Node(
        package='object_detection_cpp', executable='object_detection_node',
        name='object_detection_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_period_s': 0.125,
            'engine_path':  ParameterValue(engine_path, value_type=str),
        }])

    obj_tracker = Node(
        package='object_tracker_cpp', executable='object_tracker_node',
        name='object_tracker_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- SAFETY ----------------------------------------------------------
    safety_supervisor = Node(
        package='safety', executable='safety_supervisor_node',
        name='safety_supervisor_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_max':  0.35, 'reverse_max': 0.0,
            'deadband':     0.05, 'max_brake':   0.5,
        }])

    emergency_stop = Node(
        package='safety', executable='emergency_stop_node',
        name='emergency_stop_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time,
                     'emergency_persist_ticks': 5}])

    system_state_machine = Node(
        package='safety', executable='system_state_machine_node',
        name='system_state_machine_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'tick_period_s':       0.50,
            'ready_enter_samples': 20,
            'ready_exit_samples':  50,
        }])

    # ----- TF --------------------------------------------------------------
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

    # ----- FUSION (lifecycle) ---------------------------------------------
    ekf = LifecycleNode(
        package='ekf_localization', executable='ekf_localization_node',
        name='ekf_localization_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    drift_est = LifecycleNode(
        package='drift_estimator', executable='drift_estimator_node',
        name='drift_estimator_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'stale_state_max_s': 0.20,
            'stale_lane_max_s':  0.20,
        }])
    veh_state = LifecycleNode(
        package='vehicle_state', executable='vehicle_state_node',
        name='vehicle_state_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    tf2_pub = LifecycleNode(
        package='tf2_publisher', executable='tf2_publisher_node',
        name='tf2_publisher_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

    # ----- BEHAVIOR + CONTROL ---------------------------------------------
    behavior = LifecycleNode(
        package='behavior_planner_cpp', executable='behavior_planner_node',
        name='behavior_planner_node', namespace='', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time':           use_sim_time,
            'tick_period_s':          0.20,
            'enable_csv_logging':     False,
            'degraded_speed':         0.20,
            'degraded_persist_ticks': 10,
            'pedestrian_confirm_s':   0.15,
            'vehicle_confirm_s':      0.20,
            'sign_confirm_s':         0.15,
            'roadblock_confirm_s':    0.30,
            # Lane-follow-only mode: ignore stale-path emergency (we don't run
            # the global planner).
            'stale_path_s':           1e9,
        }])

    traj_gen = Node(
        package='trajectory_generator', executable='trajectory_generator_node',
        name='trajectory_generator_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])

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
    ui = Node(
        package='bfmc_ui_cpp', executable='bfmc_ui_node',
        name='bfmc_ui_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'publish_hz': 5.0}])
    rosbridge = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'port': 9090,
                     'address': '0.0.0.0', 'use_compression': False}])
    ui_server = Node(
        package='bfmc_ui_server', executable='bfmc_ui_server_node',
        name='bfmc_ui_server_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'port': 8080}])

    # ----- LIFECYCLE MANAGER ----------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_lanefollow', output='screen', emulate_tty=True,
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
                LogInfo(msg="[LANE-FOLLOW] Lifecycles spawned. Manager in 2s..."),
                TimerAction(period=2.0, actions=[lifecycle_manager]),
            ]))

    watchdog = Node(
        package='safety', executable='watchdog_node',
        name='watchdog_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    watchdog_trigger = TimerAction(
        period=10.0, actions=[LogInfo(msg="[LANE-FOLLOW] Watchdog…"), watchdog])

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('engine_path', default_value='',
            description='Absolute path to the TensorRT engine for object_detection_node. '
                        'Empty keeps model.engine_path from object_detection.yaml.'),

        # Drivers
        imu_driver, runtime_telemetry,
        # Perception
        perception_container, obj_det, obj_tracker,
        # Safety
        safety_supervisor, emergency_stop, system_state_machine,
        # TF
        tf_static_map_odom, tf_static_camera, tf_static_imu,
        # Fusion + behavior + control
        ekf, drift_est, veh_state, tf2_pub, behavior, traj_gen, traj_ctrl,
        # UI
        ui, rosbridge, ui_server,
        # Sequencers
        manager_trigger, watchdog_trigger,
    ])
