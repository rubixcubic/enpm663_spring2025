from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from radar_msgs.msg import RadarReturn, RadarScan
import random
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class AVSensorsInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # ==================
        # Declare parameters
        # ==================
        # parameter for the publishing rate
        self.declare_parameter("publish_rate", 2.0)
        self._publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self.declare_parameters(
            namespace="",
            parameters=[
                # camera
                (
                    "camera_name",
                    "camera",
                    ParameterDescriptor(description="Camera name"),
                ),
                (
                    "camera_rate",
                    60,
                    ParameterDescriptor(
                        description="Camera frame rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=10, to_value=60, step=1)
                        ],
                    ),
                ),
                # lidar
                (
                    "lidar_name",
                    "lidar",
                    ParameterDescriptor(description="Lidar name"),
                ),
                (
                    "lidar_rate",
                    20,
                    ParameterDescriptor(
                        description="Lidar frame rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=20, to_value=100, step=1)
                        ],
                    ),
                ),
                # radar
                (
                    "radar_name",
                    "radar",
                    ParameterDescriptor(description="Radar name"),
                ),
                (
                    "radar_rate",
                    79,
                    ParameterDescriptor(
                        description="Radar frame rate in GHz",
                        integer_range=[
                            IntegerRange(from_value=50, to_value=100, step=1)
                        ],
                    ),
                ),
            ],
        )

        # ====================================
        # Update a parameter value
        # ====================================
        self.set_parameters([Parameter("camera_rate", Parameter.Type.INTEGER, 55)])

        # ====================================
        # Get the parameters and display them
        # ====================================
        self.get_logger().info("========================")
        # ==================
        # camera name
        self._camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )
        # camera rate
        self._camera_rate = (
            self.get_parameter("camera_rate").get_parameter_value().integer_value
        )

        self.get_logger().info(
            f"\033[92m camera_name: {self._camera_name}, camera rate: {self._camera_rate} \033[0m"
        )
        # ==================
        # lidar name
        self._lidar_name = (
            self.get_parameter("lidar_name").get_parameter_value().string_value
        )
        # lidar rate
        self._lidar_rate = (
            self.get_parameter("lidar_rate").get_parameter_value().integer_value
        )

        self.get_logger().info(
            f"\033[91m lidar_name: {self._lidar_name}, lidar rate: {self._lidar_rate}  \033[0m"
        )
        # ==================
        # radar name
        self._radar_name = (
            self.get_parameter("radar_name").get_parameter_value().string_value
        )
        # radar rate
        self._radar_rate = (
            self.get_parameter("radar_rate").get_parameter_value().integer_value
        )
        self.get_logger().info(
            f"\033[94m radar_name: {self._radar_name}, radar rate: {self._radar_rate}  \033[0m"
        )

        self.get_logger().info("========================")

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_update_cb)

        # Publishers for the sensors
        self._lidar_pub = self.create_publisher(LaserScan, "/lidar/points", 10)
        self._camera_pub = self.create_publisher(Image, "/camera/image_color", 10)
        self._radar_pub = self.create_publisher(RadarScan, "/radar/tracks", 10)

        # Messages for the sensors
        self._lidar_msg = LaserScan()
        self._camera_msg = Image()
        self._radar_msg = RadarScan()

        # Timers for the sensors

        # Timer for the lidar
        self._lidar_timer = self.create_timer(self._publish_rate, self.lidar_timer_cb)
        # Timer for the camera
        self._camera_timer = self.create_timer(self._publish_rate, self.camera_timer_cb)
        # Timer for the radar
        self._radar_timer = self.create_timer(self._publish_rate, self.radar_timer_cb)

        self.get_logger().info(f"{node_name} initialized")
        self.get_logger().info("========================")

    def parameter_update_cb(self, params):
        """
        Callback function for the parameters
        """
        success = False
        for param in params:
            if param.name == "camera_name":
                if param.type_ == Parameter.Type.STRING:  # validation
                    success = True
                    if hasattr(self, "_camera_name") and self._camera_name is not None:
                        self._camera_name = param.value  # modify the attribute
                        self.get_logger().info(f"New camera_name: {self._camera_name}")
            elif param.name == "camera_rate":
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    if hasattr(self, "_camera_rate") and self._camera_rate is not None:
                        self._camera_rate = param.value
                        self.get_logger().info(f"New camera_rate: {self._camera_rate}")
            elif param.name == "publish_rate":
                if param.type_ == Parameter.Type.DOUBLE:
                    success = True
                    self._publish_rate = param.value
                    if hasattr(self, "_lidar_timer") and self._lidar_timer is not None:
                        self._lidar_timer.cancel()
                        self._lidar_timer = self.create_timer(self._publish_rate, self.lidar_timer_cb)
                        self.get_logger().info(f"New lidar_timer: {self._publish_rate}")
                    if hasattr(self, "_camera_timer") and self._camera_timer is not None:
                        self._camera_timer.cancel()
                        self._camera_timer = self.create_timer(self._publish_rate, self.camera_timer_cb)
                        self.get_logger().info(f"New camera_timer: {self._publish_rate}")
                    if hasattr(self, "_radar_timer") and self._radar_timer is not None:
                        self._radar_timer.cancel()
                        self._radar_timer = self.create_timer(self._publish_rate, self.radar_timer_cb)
                        self.get_logger().info(f"New radar_timer: {self._publish_rate}")
        return SetParametersResult(successful=success)

    def lidar_timer_cb(self):
        """
        Callback function for the lidar sensor
        """

        # Populate the LaserScan message
        self._lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self._lidar_msg.header.frame_id = "av_lidar"
        self._lidar_msg.angle_min = -3.14159
        self._lidar_msg.angle_max = 3.14159
        self._lidar_msg.angle_increment = 0.0174533
        self._lidar_msg.time_increment = 0.0
        self._lidar_msg.scan_time = 0.1
        self._lidar_msg.range_min = 0.0
        self._lidar_msg.range_max = 100.0
        self._lidar_msg.ranges = [1.0] * 360
        self._lidar_msg.intensities = [1.0] * 360
        # Publish the message
        self.get_logger().info(
            f"\033[91m Publishing message from: {self._lidar_name} \033[0m"
        )

        self._lidar_pub.publish(self._lidar_msg)

    def camera_timer_cb(self):
        """
        Callback function for the camera sensor
        """

        # Populate the Image message
        self._camera_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera_msg.header.frame_id = "av_camera"
        self._camera_msg.height = 480
        self._camera_msg.width = 640
        self._camera_msg.encoding = "rgb8"
        self._camera_msg.is_bigendian = False
        self._camera_msg.step = 640 * 3
        self._camera_msg.data = [0] * 640 * 480 * 3
        # Publish the message
        self.get_logger().info(
            f"\033[92m Publishing message from: {self._camera_name} \033[0m"
        )

        self._camera_pub.publish(self._camera_msg)

    def generate_random_radar_return(self):
        """
        Generate a random radar return message

        Returns:
            : _description_
        """
        radar_return = RadarReturn()
        radar_return.range = random.uniform(
            0.0, 100.0
        )  # Random range between 0 and 100 meters
        radar_return.azimuth = random.uniform(
            -180.0, 180.0
        )  # Random azimuth between -180 and 180 degrees
        radar_return.elevation = random.uniform(
            -90.0, 90.0
        )  # Random elevation between -90 and 90 degrees
        radar_return.doppler_velocity = random.uniform(
            -50.0, 50.0
        )  # Random Doppler velocity between -50 and 50 m/s
        radar_return.amplitude = random.uniform(
            0.0, 100.0
        )  # Random amplitude between 0 and 100
        return radar_return

    def radar_timer_cb(self):
        """
        Callback function for the radar sensor
        """

        # Create an array of RadarReturn messages
        radar_returns = []
        for _ in range(5):  # Generate 5 random radar returns
            radar_returns.append(self.generate_random_radar_return())

        # Populate the RadarScan message
        self._radar_msg.header.stamp = self.get_clock().now().to_msg()
        self._radar_msg.header.frame_id = "av_radar"
        self._radar_msg.returns = radar_returns
        # Publish the message
        self.get_logger().info(
            f"\033[94m Publishing message from: {self._radar_name} \033[0m"
        )

        self._radar_pub.publish(self._radar_msg)
