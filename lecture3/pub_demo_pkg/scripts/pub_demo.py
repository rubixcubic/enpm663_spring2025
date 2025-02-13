#!/usr/bin/env python3

import rclpy
from pub_demo_pkg.pub_demo_interface import PubDemoInterface


def main(args=None):
    rclpy.init(args=args)
    node = PubDemoInterface("pub_demo_py")
    try:
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger("pub_demo_py").error(f"Error initializing node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
