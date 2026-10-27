import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


"""
First launches gazebo (gazebo.launch.py)
Then, spawns the robot (spawn_robot.launch.py)

"""

def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    launch_dir = os.path.join(package_dir, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                package_dir, 'worlds', 'empty.sdf'
            ),
        ),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.2'),

        # ros-gz bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bridge.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
            }.items(),
        ),

        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            launch_dir, 'spawn_robot.launch.py'
                        )
                    ),
                    launch_arguments={
                        'x': LaunchConfiguration('x'),
                        'y': LaunchConfiguration('y'),
                        'z': LaunchConfiguration('z'),
                    }.items(),
                )
            ],
        ),
    ])