#!/usr/bin/env python3

import rclpy
from node_demo.node_demo_interface import NodeDemoInterface


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NodeDemoInterface("node_demo_py")
    except Exception as e:
        rclpy.logging.get_logger("node_demo_py").error(f"Error initializing node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
