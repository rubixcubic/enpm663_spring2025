#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from lifecycle_node_demo_pkg.controller_demo_interface import ControllerDemoInterface
from lifecycle_node_demo_pkg.actuator_demo_interface import ActuatorDemoInterface
from lifecycle_node_demo_pkg.nav_demo_interface import NavDemoInterface
from lifecycle_node_demo_pkg.sensor_demo_interface import SensorDemoInterface


def main():
    rclpy.init()

    # Create all Lifecycle Nodes
    sensor_node = SensorDemoInterface()
    nav_node = NavDemoInterface()
    actuator_node = ActuatorDemoInterface()
    controller_node = ControllerDemoInterface(sensor_node, nav_node, actuator_node)

    # Add nodes to executor
    executor = MultiThreadedExecutor()
    for node in [controller_node, sensor_node, nav_node, actuator_node]:
        executor.add_node(node)

    # Initial configuration
    controller_node.trigger_transition(controller_node, 'configure')

    # Spin to process commands
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Cleanup on exit
    for node in [controller_node, sensor_node, nav_node, actuator_node]:
        if node.get_state().label == 'active':
            node.deactivate()
        if node.get_state().label == 'inactive':
            node.cleanup()
        node.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    main()