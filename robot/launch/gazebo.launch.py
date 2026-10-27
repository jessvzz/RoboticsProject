import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    gazebo_dir = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(package_dir, 'worlds', 'empty.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to the Gazebo world SDF',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r ', LaunchConfiguration('world')],
            }.items(),
        ),
    ])