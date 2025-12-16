"""
Launch this file to start the internal april tag detection processes
and also launch rviz and rqt with pre-made configurations to visualize
the camera feed and the found tags. 

This launch is only for ease of debugging, and should not be used in the actual robot.
Launch start_detection.launch.py instead, for headless functioning.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    system_namespace = "apriltag_detection"

    start_detection_node = Node(
        package='april_tag_detection',
        executable='start_detection',
        name='april_tag_detector',
        namespace=system_namespace,
        output='screen'
    )

    start_mapping_srv_node = Node(
        package='april_tag_detection',
        executable='start_mapping_srv',
        name='tag_mapper',
        namespace=system_namespace,
        output='screen',
        remappings=[
            ('/tag_detection', f'/{system_namespace}/tag_detection')
        ]
    )

    # also spawn rviz and rqt for visualization

    pkg_share = get_package_share_directory('april_tag_detection')

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'turtlebot4_apriltag_debug.rviz'
    )

    print("Looking for rviz config in", rviz_config)

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # rqt image view (debug image)
    rqt_image_view = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
            '--force-discover',
            '/apriltag_detection/debug_image'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_detection_node,
        start_mapping_srv_node,
        rviz_node,
        rqt_image_view
    ])
