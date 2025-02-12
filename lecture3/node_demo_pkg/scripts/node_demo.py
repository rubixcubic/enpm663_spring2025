#!/usr/bin/env python3

import rclpy
from node_demo_pkg.node_demo_interface import NodeDemoInterface


def main(args=None):
    rclpy.init(args=args)
    node = NodeDemoInterface("node_demo_py")
    rclpy.shutdown()
    # try:
    #     rclpy.spin(node)
    # except Exception as e:
    #     rclpy.logging.get_logger("node_demo_py").error(f"Error initializing node: {e}")
    # finally:
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
