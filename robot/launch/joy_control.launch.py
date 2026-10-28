from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Read the gamepad and publish sensor_msgs/msg/Joy on /joy.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Control robot velocity and toggle the forklift using the gamepad.
        Node(
            package='robot',
            executable='robot_joy_control',
            name='robot_joy_control',
            output='screen',
        ),
    ])