# ...existing code...
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FileContent
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Locate the package that contains the URDF
    pkg_share = FindPackageShare("robot_description")

    # Path to the URDF file (place your example_robot.urdf under <pkg>/urdf/)
    urdf_file = PathJoinSubstitution(
        [pkg_share, "diff_robot.urdf"]
    )

    # Read the URDF file contents so robot_state_publisher gets the XML string
    robot_description_content = FileContent(urdf_file)

    rviz_config_file = PathJoinSubstitution([pkg_share, "robot_viewer.rviz"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",

    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
# ...existing code...