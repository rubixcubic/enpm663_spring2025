from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# This function must be defined
def generate_launch_description():
    # Declare the argument for the lidar2 topic name
    lidar1_scan_arg = DeclareLaunchArgument(
        "lidar1_scan_topic",
        default_value="lidar1_scan",
        description="The topic name to remap lidar1 scan topic",
    )

    # Declare the argument for the lidar2 topic name
    lidar2_scan_arg = DeclareLaunchArgument(
        "lidar2_scan_topic",
        default_value="lidar2_scan",
        description="The topic name to remap lidar2 scan topic",
    )
    # Use the LaunchConfiguration to get the argument value
    lidar1_scan_topic = LaunchConfiguration("lidar1_scan_topic")
    lidar2_scan_topic = LaunchConfiguration("lidar2_scan_topic")

    # Create the launch description
    ld = LaunchDescription()

    # Add the argument declaration
    ld.add_action(lidar1_scan_arg)
    ld.add_action(lidar2_scan_arg)

    # Define the nodes
    lidar1_node = Node(
        package="sensor_demo_pkg",
        executable="lidar_demo",
        name="lidar1_demo",
        remappings=[
            ("scan", lidar1_scan_topic),  # Topic remapping
        ],
    )
    lidar2_node = Node(
        package="sensor_demo_pkg",
        executable="lidar_demo",
        name="lidar2_demo",
        remappings=[
            ("scan", lidar2_scan_topic),  # Use argument for topic remapping
        ],
    )
    camera_node = Node(package="sensor_demo_pkg", executable="camera_demo")
    temperature_node = Node(package="sensor_demo_pkg", executable="temperature_demo")
    processing_node = Node(
        package="sensor_demo_pkg",
        executable="processing_demo",
        name="processing1_demo",
        remappings=[
            ("scan", lidar1_scan_topic),
        ],
    )

    # Add the nodes to the launch description
    ld.add_action(lidar1_node)
    ld.add_action(lidar2_node)
    ld.add_action(camera_node)
    ld.add_action(temperature_node)
    ld.add_action(processing_node)

    return ld
