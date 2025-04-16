import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Extract launch configurations
    use_moveit = True
    # start_rviz = LaunchConfiguration("rviz")
    start_moveit = LaunchConfiguration("moveit")
    # Set up paths
    ariac_description_pkg = get_package_share_directory("ariac_description")
    ariac_moveit_config_pkg = get_package_share_directory("ariac_moveit_config")
    ariac_tutorials_pkg = get_package_share_directory("ariac_tutorials")
    moveit_demo_pkg = get_package_share_directory("moveit_demo")

    # URDF path
    urdf_path = os.path.join(
        ariac_description_pkg, "urdf/ariac_robots/ariac_robots.urdf.xacro"
    )

    # RViz config path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("moveit_demo"), "config", "rviz_config.rviz"]
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf_path)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=os.path.join(ariac_tutorials_pkg, "config/moveit_config.yaml")
        )
        .to_moveit_configs()
    )

    # Parameters dictionary
    parameters = {"use_sim_time": True, "use_moveit": use_moveit}
    parameters.update(moveit_config.to_dict())

    # Move Group Node
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    ariac_moveit_config_pkg, "launch", "ariac_robots_moveit.launch.py"
                ),
            ]
        ),
        # condition=IfCondition(str(parameters["use_moveit"])),
        condition=IfCondition(start_moveit),
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
        # condition=IfCondition(start_rviz),
    )

    # List of nodes to start
    nodes_to_start = [move_group_launch, rviz_node]

    return nodes_to_start


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        # DeclareLaunchArgument(
        #     "rviz", default_value="false", description="Launch RViz visualization?",
        # ),
        DeclareLaunchArgument(
            "moveit", default_value="false", description="Start MoveIt?",
        ),
    ]

    # Create and return launch description
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
