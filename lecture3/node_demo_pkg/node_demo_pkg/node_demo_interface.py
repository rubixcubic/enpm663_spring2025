from rclpy.node import Node

class NodeDemoInterface(Node):
    """A simple ROS 2 node demonstration class.

    This class initializes a ROS 2 node and logs a greeting message upon creation.
    It serves as a basic example of a ROS 2 node using `rclpy.Node`.

    Attributes:
        None explicitly, but inherits from `rclpy.Node`.
    """

    def __init__(self, node_name: str):
        """Initialize the NodeDemoInterface node.

        This constructor initializes the ROS 2 node with the given name and logs 
        a message upon creation.

        Args:
            node_name (str): The name of the ROS 2 node.
        """
        super().__init__(node_name)
        self.get_logger().info(f"Hello from {self.get_name()}")
