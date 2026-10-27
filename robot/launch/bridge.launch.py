import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    bridge_config = os.path.join(
        package_dir, 'config', 'bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config,
            }],
            output='screen',
        ),
    ])