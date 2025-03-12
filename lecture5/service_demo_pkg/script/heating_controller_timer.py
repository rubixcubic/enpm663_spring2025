#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from service_demo_pkg.heating_controller_timer_interface import HeatingControllerTimer


def main(args=None):
    rclpy.init(args=args)
    node = HeatingControllerTimer("heating_controller_timer")
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
