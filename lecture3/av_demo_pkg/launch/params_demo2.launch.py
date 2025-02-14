# Pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# This function is needed
def generate_launch_description():
    ld = LaunchDescription()

    sensor_params = PathJoinSubstitution(
        [FindPackageShare("av_demo_pkg"), "config", "params.yaml"]
    )
    
    av_sensors_py = Node(
        package="av_demo_pkg",
        executable="av_sensors_demo.py",
        parameters=[sensor_params],
    )

    ld.add_action(av_sensors_py)

    return ld
