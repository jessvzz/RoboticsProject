"""
Spawns the nodes necessary to perform april tag detection. 
"""


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    start_detection_node = Node(
        package='april_tag_detection',
        executable='start_detection',
        name='april_tag_detector',
        output='screen'
    )

    start_mapping_srv_node = Node(
        package='april_tag_detection',
        executable='start_mapping_srv',
        name='tag_mapper',
        output='screen'
    )

    return LaunchDescription([
        start_detection_node,
        start_mapping_srv_node
    ])
