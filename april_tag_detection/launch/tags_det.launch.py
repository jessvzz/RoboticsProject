from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for the AprilTag Position Node."""

    return LaunchDescription([
        Node(
            package='apri_tag_detection',    
            executable='apriltag_position_node',  
            name='apriltag_position_node',
            output='screen',              # Print logs to terminal
            parameters=[                  # Optional parameters (none here for now)
                # You could add tag size, camera intrinsics as parameters if desired
                # {'tag_size_m': 0.16},
                # {'camera_fx': 554.256},
                # {'camera_fy': 554.256},
                # {'camera_cx': 320.0},
                # {'camera_cy': 240.0},
            ],
        ),
    ])
