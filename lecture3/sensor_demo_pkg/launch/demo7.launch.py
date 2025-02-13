from launch import LaunchDescription
from launch_ros.actions import Node


# This function must be defined
def generate_launch_description():
    ld = LaunchDescription()
    lidar1_node = Node(
        package="sensor_demo_pkg",
        executable="lidar_demo",
        name="lidar1_demo",
        remappings=[
            ("scan", "lidar1_scan"),  # Topic remapping
        ],
    )
    lidar2_node = Node(
        package="sensor_demo_pkg",
        executable="lidar_demo",
        name="lidar2_demo",
        remappings=[
            ("scan", "lidar2_scan"),  # Topic remapping
        ],
    )
    
    camera_node = Node(
        package="sensor_demo_pkg",
        executable="camera_demo"
    )
    
    temperature_node = Node(
        package="sensor_demo_pkg",
        executable="temperature_demo"
    )
    
    processing1_node = Node(
        package="sensor_demo_pkg",
        executable="processing_demo",
        name="processing1_demo",
        remappings=[
            ("scan", "lidar1_scan"),  # Topic remapping
        ],
    )
    
    processing2_node = Node(
        package="sensor_demo_pkg",
        executable="processing_demo",
        name="processing2_demo",
        remappings=[
            ("scan", "lidar2_scan"),  # Topic remapping
        ],
    )

    ld.add_action(lidar1_node)
    ld.add_action(lidar2_node)
    ld.add_action(camera_node)
    ld.add_action(temperature_node)
    ld.add_action(processing1_node)
    ld.add_action(processing2_node)

    return ld
