import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    sdf_file = os.path.join(package_dir, 'description', 'model.sdf')

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    rviz_config = PythonExpression([
        "'",
        os.path.join(package_dir, 'config', 'default_slam.rviz'),
        "' if '",
        LaunchConfiguration('slam'),
        "' == 'true' else '",
        os.path.join(package_dir, 'config', 'default_odom.rviz'),
        "'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Use map as RViz fixed frame',
        ),

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
    ])