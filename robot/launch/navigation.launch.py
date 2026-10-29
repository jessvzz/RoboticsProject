import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    robot_share = get_package_share_directory('robot')

    default_map = os.path.join(
        robot_share, 'map', 'map.yaml'
    )

    default_params = os.path.join(
        robot_share, 'config', 'nav2_params.yaml'
    )

    default_rviz_config = os.path.join(
        nav2_share, 'rviz', 'nav2_default_view.rviz'
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'false',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to the saved map YAML file',
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the Nav2 parameters YAML file',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time',
        ),

        nav2_launch,
        rviz_node,
    ])