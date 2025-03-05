#!/usr/bin/env python3

import rclpy
from interface_demo_pkg.interface_demo_interface import InterfaceDemoInterface


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceDemoInterface("message_interface_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()