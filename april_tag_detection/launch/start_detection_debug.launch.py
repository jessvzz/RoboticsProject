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

    pkg_share = get_package_share_directory('april_tag_detection')

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'turtlebot4_apriltag_debug.rviz'
    )

    print("Looking for rviz config in", rviz_config)

    # AprilTag detection node
    detection_node = Node(
        package='april_tag_detection',
        executable='start_detection',
        name='april_tag_detector',
        output='screen'
    )

    # Mapping / service node
    mapping_node = Node(
        package='april_tag_detection',
        executable='start_mapping_srv',
        name='tag_mapper',
        output='screen'
    )

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
        detection_node,
        mapping_node,
        rviz_node,
        rqt_image_view
    ])
