import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    launch_dir = os.path.join(package_dir, 'launch')

    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                package_dir, 'worlds', 'empty.sdf'
            ),
            description='Gazebo world file',
        ),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.2'),

        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Enable SLAM Toolbox',
        ),

        # ROS-Gazebo bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bridge.launch.py')
            )
        ),

        # RViz: choose the fixed frame based on whether SLAM is enabled
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rviz.launch.py')
            ),
            launch_arguments={
                'slam': slam,
            }.items(),
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
            }.items(),
        ),

        # Spawn robot after Gazebo starts
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

        # Start SLAM only when slam:=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'slam.launch.py')
            ),
            condition=IfCondition(slam),
        ),
    ])