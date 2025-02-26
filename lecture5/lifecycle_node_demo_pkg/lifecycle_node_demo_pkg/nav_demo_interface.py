from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class NavDemoInterface(LifecycleNode):
    def __init__(self):
        super().__init__('nav_node')
        self.cmd_vel_pub = None
        self.scan_sub = None

    def on_configure(self, state):
        self.get_logger().info("NavNode: Configuring navigation...")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("NavNode: Activating navigation...")
        self.scan_sub = self.create_subscription(
            Float32, '/scan', self.scan_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("NavNode: Deactivating navigation...")
        self.destroy_subscription(self.scan_sub)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("NavNode: Cleaning up navigation...")
        self.destroy_publisher(self.cmd_vel_pub)
        return TransitionCallbackReturn.SUCCESS

    def scan_callback(self, msg):
        twist = Twist()
        twist.linear.x = 0.5 if msg.data > 2.0 else 0.0  # Simple obstacle avoidance
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"NavNode: Published cmd_vel (x={twist.linear.x})")