#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from lifecycle_node_demo_pkg.simple_lifecycle_demo_interface import SimpleLifecycleDemo

def main():
    rclpy.init()

    # Create all Lifecycle Nodes
    simple_lifecycle_node = SimpleLifecycleDemo()
    # Add nodes to executor
    executor = MultiThreadedExecutor()
    executor.add_node(simple_lifecycle_node)

    # Spin to process commands
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Cleanup on exit
    if simple_lifecycle_node.get_state().label == 'active':
        simple_lifecycle_node.deactivate()
    if simple_lifecycle_node.get_state().label == 'inactive':
        simple_lifecycle_node.cleanup()
    simple_lifecycle_node.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    main()