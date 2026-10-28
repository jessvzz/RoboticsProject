import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    robot_dir = get_package_share_directory('robot')

    slam_launch_file = os.path.join(
        slam_toolbox_dir,
        'launch',
        'online_async_launch.py'
    )

    slam_params_file = os.path.join(
        robot_dir,
        'config',
        'slam.yaml'
    )

    filter_params_file = os.path.join(
        robot_dir,
        'config',
        'scan_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            parameters=[filter_params_file],
            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan'),
            ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params_file,
            }.items(),
        ),
    ])