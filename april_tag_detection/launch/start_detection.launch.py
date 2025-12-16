"""
Spawns the nodes necessary to perform april tag detection. 
"""


from launch import LaunchDescription
from launch_ros.actions import Node


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
        executable='start_tag_mapping',
        name='tag_mapper',
        namespace=system_namespace,
        output='screen',
        remappings=[
            ('/tag_detection', f'/{system_namespace}/tag_detection')
        ]
    )

    return LaunchDescription([
        start_detection_node,
        start_mapping_srv_node
    ])
