from launch import LaunchDescription
from launch_ros.actions import Node


# This function must be defined
def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
            ),
        ]
    )