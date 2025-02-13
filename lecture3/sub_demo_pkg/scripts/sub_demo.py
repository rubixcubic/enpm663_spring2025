#!/usr/bin/env python3

"""
ROS 2 Subscriber Node Entry Point.

This script initializes and runs a ROS 2 subscriber node using the `SubDemoInterface` class. 
It sets up the ROS 2 environment, creates the subscriber node, and manages its lifecycle.

If an exception occurs during execution, an error message is logged. The node is 
properly destroyed and ROS 2 is shut down before exiting.

Author: Zeid Kootbally
"""

import rclpy
from sub_demo_pkg.sub_demo_interface import SubDemoInterface


def main(args=None):
    """Initialize and run the ROS 2 subscriber node.

    This function initializes the ROS 2 system, creates an instance of `SubDemoInterface`, 
    and spins the node to keep it running. If an exception occurs, it logs an error message. 
    The function ensures that the node is properly destroyed and ROS 2 is shut down before exiting.

    Args:
        args (list, optional): Command-line arguments for ROS 2 initialization. Defaults to None.
    """
    rclpy.init(args=args)
    node = SubDemoInterface("sub_demo_py")
    try:
        rclpy.spin(node)  # Keeps the node active to listen for messages
    except Exception as e:
        rclpy.logging.get_logger("sub_demo_py").error(f"Error initializing node: {e}")
    finally:
        node.destroy_node()  # Clean up the node before shutting down
        rclpy.shutdown()  # Properly shutdown ROS 2

if __name__ == "__main__":
    main()
