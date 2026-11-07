# Race-day manual drive mode.
#
# Brings up: drivers + perception + fusion + UI + teleop bridge + rosbag2.
# NOT brought up: behavior planner, mission supervisor, NMPC trajectory
# controller, safety supervisor — manual mode talks directly to the
# /control/command channel via the teleop_to_control_node bridge.
#
# Operator drives with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
# in a separate terminal:
#   sources WASD → /cmd_vel (geometry_msgs/Twist)
#   teleop_to_control_node →  /control/command (bfmc_msgs/ControlCommand)
#   throttle clamped [0, 0.35], no reverse possible, brake clamped [0, 0.45].
#
# Rosbag2 records every relevant topic into ~/bfmc_runs/run_NNN — NNN auto-
# increments per run by scanning the dir at launch time.

import os
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

_dds_xml = os.path.join(
    get_package_share_directory('system_bringup'), 'config', 'cyclonedds_race.xml')


# ---------- topics recorded per run -----------------------------------------
RECORDED = [
    # raw + filtered sensors
    '/microros/imu_raw',
    '/imu/data_filtered',
    '/camera/camera/color/image_raw',
    '/camera/camera/depth/image_rect_raw',
    '/camera/camera/color/camera_info',
    # perception
    '/perception/image_preprocessed',
    '/perception/lane_geometry',
    '/perception/free_space',
    '/perception/objects',
    '/perception/tracked_objects',
    # fusion + localization
    '/vehicle/state',
    '/vehicle/state_raw',
    '/fusion/imu_confidence',
    '/fusion/yaw_bias_correction',
    # teleop chain
    '/cmd_vel',
    '/control/command',
    # safety / system
    '/safety/emergency',
    '/safety/emergency_stop',
    '/system/runtime_health',
    # TF
    '/tf', '/tf_static',
]


def _next_run_dir(base_dir: str) -> str:
    """Scan base_dir for run_NNN/, pick next free NNN, return full path."""
    os.makedirs(base_dir, exist_ok=True)
    pat = re.compile(r'^run_(\d+)$')
    next_n = 1
    for name in os.listdir(base_dir):
        m = pat.match(name)
        if m:
            n = int(m.group(1))
            if n >= next_n:
                next_n = n + 1
    return os.path.join(base_dir, f'run_{next_n:03d}')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # TensorRT engine for object_detection_node. Empty = use model.engine_path
    # from object_detection.yaml.
    engine_path = LaunchConfiguration('engine_path', default='')
    bag_base = LaunchConfiguration('bag_base',
                                   default=os.path.expanduser('~/bfmc_runs'))

    # The directory must be resolved at launch description build time so
    # rosbag2 gets a concrete `-o` argument. We resolve against the default
    # path; operators overriding `bag_base:=...` should also pass
    # `run_dir:=...` explicitly.
    resolved_run_dir = _next_run_dir(os.path.expanduser('~/bfmc_runs'))

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

    # ----- FUSION (lifecycle, brought up by manager below) -----------------
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

    # ----- TELEOP BRIDGE ---------------------------------------------------
    teleop_bridge = Node(
        package='bfmc_control', executable='teleop_to_control_node',
        name='teleop_to_control_node', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_throttle': 0.35,
            'max_brake':    0.45,
            'max_steer_rad': 0.436,
            'max_speed_mps': 0.50,
            'deadband':      0.05,
            'publish_hz':    50.0,
            'cmd_timeout_s': 0.5,
        }])

    # ----- UI --------------------------------------------------------------
    ui = Node(
        package='bfmc_ui_cpp', executable='bfmc_ui_node',
        name='bfmc_ui_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'publish_hz': 5.0}])

    rosbridge = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time, 'port': 9090,
            'address': '0.0.0.0', 'use_compression': False,
        }])

    ui_server = Node(
        package='bfmc_ui_server', executable='bfmc_ui_server_node',
        name='bfmc_ui_server_node', output='screen', emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'port': 8080}])

    # ----- LIFECYCLE MANAGER ----------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_manual', output='screen', emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'lane_geometry_node', 'free_space_node',
                'ekf_localization_node', 'drift_estimator_node',
                'vehicle_state_node', 'tf2_publisher_node',
            ]},
            {'bond_timeout': 0.0},
            {'attempt_respawn_reconnection': False},
        ])

    # ----- ROSBAG2 ---------------------------------------------------------
    # `-o run_NNN` is computed at launch-description time. The bag starts a
    # few seconds after launch so it doesn't race with composable container
    # startup.
    rosbag_cmd = (
        ['ros2', 'bag', 'record', '-o', resolved_run_dir,
         '--storage', 'sqlite3',
         '--compression-mode', 'none']
        + RECORDED)
    rosbag_proc = ExecuteProcess(
        cmd=rosbag_cmd, output='screen',
        log_cmd=True, shell=False)
    rosbag_trigger = TimerAction(period=5.0, actions=[
        LogInfo(msg=f"[MANUAL] Recording rosbag → {resolved_run_dir}"),
        rosbag_proc,
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('engine_path', default_value='',
            description='Absolute path to the TensorRT engine for object_detection_node. '
                        'Empty keeps model.engine_path from object_detection.yaml.'),
        DeclareLaunchArgument('bag_base',
            default_value=os.path.expanduser('~/bfmc_runs'),
            description='Base directory for run_NNN/ rosbag2 dirs.'),

        # Drivers
        imu_driver, runtime_telemetry,
        # Perception
        perception_container, obj_det, obj_tracker,
        # Fusion + TF
        ekf, drift_est, veh_state, tf2_pub,
        tf_static_map_odom, tf_static_camera, tf_static_imu,
        # Teleop bridge (Twist → ControlCommand)
        teleop_bridge,
        # UI
        ui, rosbridge, ui_server,
        # Lifecycle manager
        lifecycle_manager,
        # Rosbag (delayed 5 s)
        rosbag_trigger,
    ])
