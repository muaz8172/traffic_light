import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_name = 'traffic_light'
    node_name = 'traffic_light'

    package_share = get_package_share_directory(package_name)

    parameter_file = os.path.join(
        package_share,
        'config',
        'traffic_light.yaml',
    )

    node = Node(
        package=package_name,
        executable='traffic_light_node',
        name=node_name,
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            parameter_file,
        ],
    )

    return LaunchDescription([
        node,
    ])
