import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Create parameters dictionary
    parameters = {"use_sim_time": True}
    parameters["use_moveit"] = True

    urdf = os.path.join(
        get_package_share_directory("ariac_description"),
        "urdf/ariac_robots/ariac_robots.urdf.xacro",
    )

    # RViz config path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("moveit_demo"), "config", "rviz_config.rviz"]
    )

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("moveit_demo")
            + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )

    start_rviz = LaunchConfiguration("rviz")

    parameters.update(moveit_config.to_dict())

    # Tutorial Node
    moveit_demo_node = Node(
        package="moveit_demo",
        executable="minimal_demo.py",
        output="screen",
        parameters=[parameters],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_rviz),
    )

    # Move Group node
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        ),
        condition=IfCondition(str(parameters["use_moveit"])),
    )

    nodes_to_start = [
        moveit_demo_node,
        move_group,
        rviz_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz visualization?",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
