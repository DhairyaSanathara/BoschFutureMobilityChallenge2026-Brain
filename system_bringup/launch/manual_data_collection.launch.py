from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

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
                extra_arguments=[
                    {'use_intra_process_comms':True}
                ]
            ),

            ComposableNode(
                package='lane_geometry_cpp',
                plugin='lane_geometry_cpp::LaneGeometryNode',
                name='lane_geometry_node',
                extra_arguments=[
                    {'use_intra_process_comms':True}
                ]
            ),

            ComposableNode(
                package='free_space_cpp',
                plugin='free_space_cpp::FreeSpaceNode',
                name='free_space_node',
                extra_arguments=[
                    {'use_intra_process_comms':True}
                ]
            )
        ],
        output='screen'
    )


    rosbridge=Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port':9090
        }],
        output='screen'
    )


    imu_driver=Node(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver_node',
        output='screen'
    )


    return LaunchDescription([

        imu_driver,

        perception_container,

        rosbridge
    ])
