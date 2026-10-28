import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, package_dir, launch_dir):
    slam = LaunchConfiguration('slam').perform(context)
    world_name = LaunchConfiguration('world').perform(context)

    # Resolve the world path.
    # Accepts: warehouse, warehouse.sdf, or an absolute path.
    if os.path.isabs(world_name):
        world_path = world_name
        if not world_path.endswith('.sdf'):
            world_path += '.sdf'
    else:
        if not world_name.endswith('.sdf'):
            world_name += '.sdf'

        world_path = os.path.join(package_dir, 'worlds', world_name)

    if not os.path.isfile(world_path):
        raise FileNotFoundError(
            f"World file not found: {world_path}"
        )

    return [
        # ROS-Gazebo bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bridge.launch.py')
            )
        ),

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rviz.launch.py')
            ),
            launch_arguments={'slam': slam}.items(),
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # Spawn robot after Gazebo starts
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'spawn_robot.launch.py')
                    ),
                    launch_arguments={
                        'x': LaunchConfiguration('x'),
                        'y': LaunchConfiguration('y'),
                        'z': LaunchConfiguration('z'),
                    }.items(),
                )
            ],
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'slam.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    launch_dir = os.path.join(package_dir, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_world',
            description=(
                'World name (e.g. warehouse), filename (warehouse.sdf), '
                'or absolute path to an SDF file'
            ),
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.2'),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Enable SLAM Toolbox',
        ),
        OpaqueFunction(
            function=launch_setup,
            kwargs={
                'package_dir': package_dir,
                'launch_dir': launch_dir,
            },
        ),
    ])
