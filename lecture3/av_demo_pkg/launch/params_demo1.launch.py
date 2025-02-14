# Pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node


# This function is needed
def generate_launch_description():
    ld = LaunchDescription()

    av_sensors_py = Node(
        package="av_demo_pkg",
        executable="av_sensors_demo.py",
        parameters=[
            {"camera_name": "front_camera"},
            {"lidar_name": "roof_lidar"},
            {"radar_name": "front_bumper_radar"},
            {"camera_rate": 20},
            {"lidar_rate": 30},
            {"radar_rate": 60},
        ],
    )

    ld.add_action(av_sensors_py)

    return ld
