import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    package_name = 'traffic_light'
    node_name = 'traffic_light'

    package_share = get_package_share_directory(package_name)

    parameter_file = os.path.join(
        package_share,
        'config',
        'traffic_light.yaml',
    )

    component = ComposableNode(
        package=package_name,
        plugin='traffic_light::TrafficLightComponent',
        name=node_name,
        namespace='',
        parameters=[
            parameter_file,
        ],
        extra_arguments=[
            {
                'use_intra_process_comms': True,
            },
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='traffic_light_container',
        namespace='',
        output='screen',
        emulate_tty=True,
        composable_node_descriptions=[
            component,
        ],
    )

    return LaunchDescription([
        container,
    ])
