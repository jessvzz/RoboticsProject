import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('robot')
    sdf_file = os.path.join(package_dir, 'description', 'model.sdf')
    
    mode = LaunchConfiguration('mode').perform(context).lower()

    MODES = {
        'slam' : 'default_slam.rviz',
        'odom' : 'default_odom.rviz',
        'nav'  : 'default_nav.rviz'
    }

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        package_dir,
        'config',
        MODES[mode]
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='odom',
            description='Select visualization mode: slam, odom or nav',
        ),

        OpaqueFunction(function=launch_setup),
    ])