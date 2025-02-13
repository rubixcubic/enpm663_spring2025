from launch import LaunchDescription
from launch_ros.actions import Node


# This function must be defined
def generate_launch_description():
    ld = LaunchDescription()
    talker = Node(
        package="demo_nodes_cpp", 
        executable="talker")
    listener = Node(
        package="demo_nodes_cpp", 
        executable="listener")

    ld.add_action(talker)
    ld.add_action(listener)

    return ld