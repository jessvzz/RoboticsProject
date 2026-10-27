import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params_file,
            }.items(),
        ),
    ])