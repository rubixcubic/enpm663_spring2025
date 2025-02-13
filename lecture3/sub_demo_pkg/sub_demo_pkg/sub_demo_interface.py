from rclpy.node import Node
from std_msgs.msg import String


class SubDemoInterface(Node):
    """A ROS 2 subscriber node for the 'leia' topic.

    This class creates a ROS 2 node that subscribes to the 'leia' topic and processes 
    incoming messages of type `std_msgs.msg.String`. Messages received on this topic 
    are logged using the node's built-in logging mechanism.
    """

    def __init__(self, node_name: str):
        """Initialize the SubDemoInterface node.

        This constructor initializes the ROS 2 node with the given name and creates a 
        subscription to the 'leia' topic. The `leia_topic_cb` callback is registered 
        to handle incoming messages.

        Args:
            node_name (str): The name of the ROS 2 node.
        """
        super().__init__(node_name)
        self._leia_sub = self.create_subscription(
            String, "leia", self.leia_topic_cb, 10
        )

    def leia_topic_cb(self, msg: String):
        """Handle incoming messages on the 'leia' topic.

        This function is triggered when a new message is received on the 'leia' topic.
        It logs the message content using the node's logger.

        Args:
            msg (std_msgs.msg.String): The received message object containing the string data.
        """
        self.get_logger().info(f"Receiving: {msg.data}")
        # Logs the received message data.
