from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from geometry_msgs.msg import Twist

class ActuatorDemoInterface(LifecycleNode):
    def __init__(self):
        super().__init__('actuator_node')
        self._cmd_sub = None

    def on_configure(self, state):
        self.get_logger().info("ActuatorNode: Configuring actuators...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("ActuatorNode: Activating actuators...")
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("ActuatorNode: Deactivating actuators...")
        self.destroy_subscription(self._cmd_sub)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("ActuatorNode: Cleaning up actuators...")
        return TransitionCallbackReturn.SUCCESS

    def cmd_callback(self, msg):
        self.get_logger().info(f"ActuatorNode: Moving at speed {msg.linear.x}")