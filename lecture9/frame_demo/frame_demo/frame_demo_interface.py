
import rclpy
import PyKDL
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ariac_msgs.msg import (
    AdvancedLogicalCameraImage as AriacAdvancedLogicalCameraImage,
    Part as AriacPart,
)
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from tf_transformations import euler_from_quaternion


class BroadcasterDemo(Node):
    """
    Class to broadcast transforms for parts detected by cameras in ROS2.

    This class demonstrates both static and dynamic broadcasting of transforms.
    It subscribes to logical camera feeds from left and right bins, searches for
    a specific part (purple pump), and broadcasts its transform when found.

    The class can also optionally listen for transforms between frames if the
    'listen' parameter is set to True.

    Attributes:
        _listen_param (bool): Whether to listen for transforms or not
        _left_bin_parts (list): List of parts detected in the left bin
        _right_bin_parts (list): List of parts detected in the right bin
        _found_purple_pump (bool): Flag indicating if purple pump is found
        _part_parent_frame (str): Parent frame of the detected part
        _part_frame (str): Name of the frame for the detected part
        _part_pose (Pose): Pose of the detected part
        _find_part_color (int): Color code for the part to find (PURPLE)
        _find_part_type (int): Type code for the part to find (PUMP)
        _transforms (list): List of transforms to be broadcast
    """

    def __init__(self, node_name):
        """
        Initialize the BroadcasterDemo node.

        Args:
            node_name (str): Name of the ROS node
        """
        super().__init__(node_name)

        # Enable simulation time for this node
        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # Get the listen parameter from ROS parameters
        self._listen_param = (
            self.declare_parameter("listen", False).get_parameter_value().bool_value
        )

        # Initialize lists to store detected parts in each bin
        # These lists contain AdvancedLogicalCameraImage objects
        self._left_bin_parts = []
        self._right_bin_parts = []

        # Initialize state variables for part detection and broadcasting
        self._found_purple_pump = False  # Flag to check if target part is found
        self._part_parent_frame = None  # Frame in which part was detected
        self._part_frame = "purple_pump_1"  # Name for the part's frame
        self._part_pose = None  # Pose of the detected part

        # Define target part specifications
        self._find_part_color = AriacPart.PURPLE
        self._find_part_type = AriacPart.PUMP

        # Create subscriber to process left bin camera images
        self._left_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/left_bins_camera/image",
            self.left_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # Create subscriber to process right bin camera images
        self._right_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/right_bins_camera/image",
            self.right_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # Create timer to periodically search for the target part
        self._find_part_timer = self.create_timer(0.05, self.find_part_callback)

        # List to store transforms that need to be broadcast
        self._transforms = []

        # Create a dynamic broadcaster for publishing transforms
        self._tf_dynamic_broadcaster = TransformBroadcaster(self)

        # If listening is enabled, set up transform listener components
        if self._listen_param:
            # Create buffer and listener for transforms
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

            # Create timer to periodically check transforms
            self._listener_timer = self.create_timer(0.5, self._listener_cb)

        self.get_logger().info("Broadcaster demo started")

    def find_part_callback(self):
        """
        Periodically search for the target part (purple pump) in both bins.

        This callback is triggered by the find_part_timer. It searches through
        the detected parts in both left and right bins, looking for a purple pump.
        Once found, it generates a transform for the part and begins broadcasting.
        """

        if not self._found_purple_pump:
            self.get_logger().info("Searching...")

            # Check if purple pump is in the left bin
            for part_pose in self._left_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._part_parent_frame = "left_bins_camera_frame"
                    self._part_pose = part_pose.pose
                    self._found_purple_pump = True
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame, self._part_pose
                    )
                    break

            # If not found in left bin, check the right bin
            for part_pose in self._right_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._part_parent_frame = "right_bins_camera_frame"
                    self._part_pose = part_pose.pose
                    self._found_purple_pump = True
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame, self._part_pose
                    )
                    break
        else:
            # If purple pump was already found, broadcast its transform
            self.broadcast()

    def broadcast(self):
        """
        Publish all transforms in the transform list.

        Uses the dynamic broadcaster to send all transforms that have been
        generated and stored in the _transforms list.
        """
        self._tf_dynamic_broadcaster.sendTransform(self._transforms)

    def left_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Process images from the left bins camera.

        Updates the list of parts detected in the left bin based on
        the received camera image.

        Args:
            msg (AriacAdvancedLogicalCameraImage): Camera image message
        """
        # Clear previous detections
        self._left_bin_parts.clear()

        # Check if any parts were detected
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in left bins")
            return

        # Store all detected parts
        for part_pose in msg.part_poses:
            # Uncomment to log each detected part
            # self.get_logger().info(
            #     f"Part detected in left bins: {part_pose.part.type} {part_pose.part.color}"
            # )
            self._left_bin_parts.append(part_pose)

    def right_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Process images from the right bins camera.

        Updates the list of parts detected in the right bin based on
        the received camera image.

        Args:
            msg (AriacAdvancedLogicalCameraImage): Camera image message
        """
        # Clear previous detections
        self._right_bin_parts.clear()

        # Check if any parts were detected
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in right bins")
            return

        # Store all detected parts
        for part_pose in msg.part_poses:
            # Uncomment to log each detected part
            # self.get_logger().info(
            #     f"Part detected in right bins: {part_pose.part.type} {part_pose.part.color}"
            # )
            self._right_bin_parts.append(part_pose)

    def generate_transform(self, parent, child, pose):
        """
        Build a transform message and add it to the list of transforms.

        Creates a TransformStamped message based on the provided parent frame,
        child frame, and pose, then adds it to the list of transforms to be broadcast.

        Args:
            parent (str): Parent frame ID
            child (str): Child frame ID
            pose (geometry_msgs.msg.Pose): Pose of child frame relative to parent
        """
        transform_stamped = TransformStamped()

        # Set header information
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        # Set translation from pose position
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z

        # Set rotation from pose orientation
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w

        # Add the transform to the list
        self._transforms.append(transform_stamped)

    def _listener_cb(self):
        """
        Listen for transforms between frames.

        This callback is triggered by the listener timer when the 'listen'
        parameter is True. It attempts to look up the transform between the
        'world' frame and the part frame, then logs the result.
        """
        try:
            # Check if part parent frame is set
            if self._part_parent_frame is None:
                self.get_logger().warn("Part parent frame is not set.")
                return

            # Look up transform between world frame and part frame
            transform = self._tf_buffer.lookup_transform(
                "world", self._part_frame, rclpy.time.Time()
            )

            # Log the transform information
            self.get_logger().info(
                f"Transform between world and {self._part_frame}: \n" + str(transform)
            )
        except TransformException as ex:
            # Log error if transform lookup fails
            self.get_logger().fatal(
                f"Could not get transform between world and {self._part_frame}: {str(ex)}"
            )


class KDLFrameDemo(Node):
    """
    Class to demonstrate frame transformations using KDL in ROS2.

    This class subscribes to camera feeds, detects parts, and computes
    their poses in the world coordinate frame using the KDL library.
    It specifically searches for a purple pump and transforms its pose
    from camera coordinates to world coordinates.

    Attributes:
        _left_bin_parts (list): List of parts detected in the left bin
        _right_bin_parts (list): List of parts detected in the right bin
        _found_purple_pump (bool): Flag indicating if purple pump is found
        _find_part_color (int): Color code for the part to find (PURPLE)
        _find_part_type (int): Type code for the part to find (PUMP)
        _right_bins_camera_pose_in_world (Pose): Pose of right bins camera in world frame
        _left_bins_camera_pose_in_world (Pose): Pose of left bins camera in world frame
        _part_pose_in_world (Pose): Computed pose of detected part in world frame
    """

    def __init__(self, node_name):
        """
        Initialize the KDLFrameDemo node.

        Args:
            node_name (str): Name of the ROS node
        """
        super().__init__(node_name)

        # Enable simulation time for this node
        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # Initialize lists to store detected parts in each bin
        # These lists contain AdvancedLogicalCameraImage objects
        self._left_bin_parts = []
        self._right_bin_parts = []

        # Flag to check if the target part is found
        self._found_purple_pump = False

        # Define target part specifications
        self._find_part_color = AriacPart.PURPLE
        self._find_part_type = AriacPart.PUMP

        # Initialize camera and part pose variables
        self._right_bins_camera_pose_in_world = None
        self._left_bins_camera_pose_in_world = None
        self._part_pose_in_world = None

        # Create subscriber to process left bin camera images
        self._left_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/left_bins_camera/image",
            self.left_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # Create subscriber to process right bin camera images
        self._right_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/right_bins_camera/image",
            self.right_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # Create timer to periodically search for the target part
        self._find_part_timer = self.create_timer(0.05, self.find_part_callback)

        self.get_logger().info("KDL frame demo started")

    def find_part_callback(self):
        """
        Periodically search for the target part (purple pump) in both bins.

        This callback is triggered by the find_part_timer (every 0.05 seconds).
        It searches through the detected parts in both bins, looking for a purple pump.
        Once found, it computes the part's pose in the world coordinate frame.
        """

        if not self._found_purple_pump:
            self.get_logger().info("Searching...")

            # Check if purple pump is in the left bin
            for part_pose in self._left_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._found_purple_pump = True
                    # Transform part pose from camera frame to world frame
                    self._part_pose_in_world = self.compute_part_pose_in_world(
                        part_pose.pose, self._left_bins_camera_pose_in_world
                    )
                    break

            # If not found in left bin, check the right bin
            for part_pose in self._right_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._found_purple_pump = True
                    # Transform part pose from camera frame to world frame
                    self._part_pose_in_world = self.compute_part_pose_in_world(
                        part_pose.pose, self._right_bins_camera_pose_in_world
                    )
                    break

    def compute_part_pose_in_world(self, part_pose_in_camera, camera_pose_in_world):
        """
        Transform a part's pose from camera frame to world frame.

        This function performs a coordinate frame transformation using KDL frames.
        It computes the pose of a part in the world coordinate frame given its pose
        in the camera frame and the camera's pose in the world frame.

        The transformation is performed in the following steps:
        1. Create a KDL frame for the camera pose in world frame
        2. Create a KDL frame for the part pose in camera frame
        3. Calculate the part pose in world frame by multiplying the two frames
        4. Convert the resulting KDL frame back to a Pose message

        Args:
            part_pose_in_camera (Pose): The pose of the part in the camera coordinate frame
            camera_pose_in_world (Pose): The pose of the camera in the world coordinate frame

        Returns:
            Pose: The pose of the part in the world coordinate frame

        Note:
            This function logs the resulting part position and orientation (in roll-pitch-yaw format)
            to the ROS logger at INFO level.
        """
        # 1. Create KDL Frame for camera-to-world transformation
        camera_orientation = camera_pose_in_world.orientation
        camera_position = camera_pose_in_world.position
        kdl_camera_world = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_position.x, camera_position.y, camera_position.z),
        )

        # 2. Create KDL Frame for part-to-camera transformation
        part_orientation = part_pose_in_camera.orientation
        part_position = part_pose_in_camera.position
        kdl_part_camera = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            PyKDL.Vector(part_position.x, part_position.y, part_position.z),
        )

        # 3. Compute part-to-world transformation by composing the two frames
        # This multiplication performs the actual coordinate transformation
        kdl_part_world = kdl_camera_world * kdl_part_camera

        # 4. Convert the KDL frame back to a Pose message
        pose = Pose()
        pose.position.x = kdl_part_world.p.x()
        pose.position.y = kdl_part_world.p.y()
        pose.position.z = kdl_part_world.p.z()

        # Extract quaternion from rotation matrix
        q = kdl_part_world.M.GetQuaternion()

        # Set the orientation in the pose message (not shown in the original code)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Log the resulting pose information with roll-pitch-yaw representation
        rpy = euler_from_quaternion(q, "sxyz")
        output = "\n" + "=" * 50 + "\n"
        output += f"Part position in world frame: \n x: {pose.position.x:.4f}, y: {pose.position.y:.4f}, z: {pose.position.z:.4f}\n"
        output += f"Part orientation in world frame: \n rpy: {rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}\n"
        output += "=" * 50 + "\n"
        self.get_logger().info(output)

        return pose

    def left_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Process images from the left bins camera.

        Updates the list of parts detected in the left bin and stores
        the camera's pose in world frame if not already set.

        Args:
            msg (AriacAdvancedLogicalCameraImage): Camera image message containing
                                                  part poses and sensor pose
        """
        # Clear previous detections
        self._left_bin_parts.clear()

        # Check if any parts were detected
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in left bins")
            return

        # Store camera pose in world frame if not already set
        if self._left_bins_camera_pose_in_world is None:
            self._left_bins_camera_pose_in_world = msg.sensor_pose

        # Store all detected parts
        for part_pose in msg.part_poses:
            self._left_bin_parts.append(part_pose)

    def right_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Process images from the right bins camera.

        Updates the list of parts detected in the right bin and stores
        the camera's pose in world frame if not already set.

        Args:
            msg (AriacAdvancedLogicalCameraImage): Camera image message containing
                                                  part poses and sensor pose
        """
        # Clear previous detections
        self._right_bin_parts.clear()

        # Check if any parts were detected
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in right bins")
            return

        # Store camera pose in world frame if not already set
        if self._right_bins_camera_pose_in_world is None:
            self._right_bins_camera_pose_in_world = msg.sensor_pose

        # Store all detected parts
        for part_pose in msg.part_poses:
            self._right_bin_parts.append(part_pose)
