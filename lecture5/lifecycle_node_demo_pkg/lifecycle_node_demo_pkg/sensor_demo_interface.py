from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32

class SensorDemoInterface(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.scan_pub = None

    def on_configure(self, state):
        self.get_logger().info("SensorNode: Configuring sensors...")
        self.scan_pub = self.create_publisher(Float32, '/scan', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("SensorNode: Activating sensors...")
        self.timer = self.create_timer(1.0, self.scan_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("SensorNode: Deactivating sensors...")
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("SensorNode: Cleaning up sensors...")
        self.destroy_publisher(self.scan_pub)
        return TransitionCallbackReturn.SUCCESS

    def scan_callback(self):
        msg = Float32()
        msg.data = 10.0  # Simulated distance reading
        self.scan_pub.publish(msg)
        self.get_logger().info("SensorNode: Publishing scan data")