#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from service_demo_pkg.heating_controller_direct_interface import HeatingControllerDirect


def main(args=None):
    rclpy.init(args=args)
    node = HeatingControllerDirect("heating_controller_direct")
    # create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
