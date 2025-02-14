# Import necessary Python launch modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


# Function to generate the launch description
def generate_launch_description():
    ld = LaunchDescription()

    # Declare a command-line argument "publish_rate"
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="2.0",  # Default value must be a string
        description="Argument to set the publish rate",
    )

    # Define the path to the parameter file
    sensor_params = PathJoinSubstitution(
        [FindPackageShare("av_demo_pkg"), "config", "params.yaml"]
    )

    # Define the node with parameters
    av_sensors_py = Node(
        package="av_demo_pkg",
        executable="av_sensors_demo.py",
        parameters=[
            {"publish_rate": LaunchConfiguration("publish_rate")},
            sensor_params,
        ],
    )

    # Add actions to the launch description
    ld.add_action(publish_rate_arg)
    ld.add_action(av_sensors_py)

    return ld
