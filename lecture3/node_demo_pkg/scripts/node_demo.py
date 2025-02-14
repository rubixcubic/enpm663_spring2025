#!/usr/bin/env python3

"""
ROS 2 Node Entry Point.

This script initializes and runs a ROS 2 node using the `NodeDemoInterface` class.
It sets up the ROS 2 environment, creates a subscriber or publisher node, and 
properly shuts down ROS 2 before exiting.

Author: Zeid Kootbally
"""

import rclpy
from node_demo_pkg.node_demo_interface import NodeDemoInterface


def main(args=None):
    """Initialize and run the ROS 2 node.

    This function initializes the ROS 2 system, creates an instance of 
    `NodeDemoInterface`, and shuts down ROS 2 after execution. 

    Note:
        - `rclpy.spin(node)` is currently commented out, so the node does 
          not actively process callbacks.
        - Uncomment `rclpy.spin(node)` to keep the node running.
    
    Args:
        args (list, optional): Command-line arguments for ROS 2 initialization. 
                               Defaults to None.
    """
    rclpy.init(args=args)
    node = NodeDemoInterface("node_demo_py")
    rclpy.shutdown()

    # Uncomment the following block to enable node execution:
    # try:
    #     rclpy.spin(node)  # Keeps the node running
    # except Exception as e:
    #     rclpy.logging.get_logger("node_demo_py").error(f"Error initializing node: {e}")
    # except KeyboardInterrupt:
    #     # Log a message when the node is manually terminated
    #     node.get_logger().warn("Keyboard interrupt detected")
    # finally:
    #     node.get_logger().warn("Exited spin()")
    #     rclpy.shutdown()

if __name__ == "__main__":
    main()
