from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    ld = LaunchDescription()

    # Grouped nodes
    navigation_group = GroupAction([
        Node(package='sensor_demo_pkg', executable='lidar_demo'),
        Node(package='sensor_demo_pkg', executable='camera_demo')
    ])
    
    temperature_node = Node(package='sensor_demo_pkg', executable='temperature_demo')

    ld.add_action(navigation_group)
    ld.add_action(temperature_node)
    return ld