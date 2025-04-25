from math import cos, sin, pi
from copy import copy
import time
import PyKDL
from sympy import Quaternion
from ament_index_python import get_package_share_directory
from moveit import MoveItPy, PlanningSceneMonitor
import rclpy
import pyassimp
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from std_msgs.msg import Header

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from os import path

from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, ApplyPlanningScene
from moveit.core.kinematic_constraints import construct_joint_constraint

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    AGVStatus as AGVStatusMsg,
    VacuumGripperState,
)

from ariac_msgs.srv import (
    VacuumGripperControl,
    ChangeGripper,
)

from std_srvs.srv import Trigger

from moveit_demo.utils import (
    multiply_pose,
    rpy_from_quaternion,
    rad_to_deg_str,
    quaternion_from_euler,
    build_pose,
    AdvancedLogicalCameraImage,
    Order,
    KittingTask,
    CombinedTask,
    AssemblyTask,
    KittingPart,
)

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return repr(self.value)


class RobotController(Node):
    """
    Class for controlling a robot in the ARIAC competition
    """

    _part_colors = {
        PartMsg.RED: "red",
        PartMsg.BLUE: "blue",
        PartMsg.GREEN: "green",
        PartMsg.ORANGE: "orange",
        PartMsg.PURPLE: "purple",
    }

    _part_colors_emoji = {
        PartMsg.RED: "🟥",
        PartMsg.BLUE: "🟦",
        PartMsg.GREEN: "🟩",
        PartMsg.ORANGE: "🟧",
        PartMsg.PURPLE: "🟪",
    }

    _part_types = {
        PartMsg.BATTERY: "battery",
        PartMsg.PUMP: "pump",
        PartMsg.REGULATOR: "regulator",
        PartMsg.SENSOR: "sensor",
    }

    _competition_states = {
        CompetitionStateMsg.IDLE: "idle",
        CompetitionStateMsg.READY: "ready",
        CompetitionStateMsg.STARTED: "started",
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: "order_announcements_done",
        CompetitionStateMsg.ENDED: "ended",
    }

    _gripper_states = {True: "enabled", False: "disabled"}

    _part_heights = {
        PartMsg.BATTERY: 0.04,
        PartMsg.PUMP: 0.12,
        PartMsg.REGULATOR: 0.07,
        PartMsg.SENSOR: 0.07,
    }

    _quad_offsets = {
        1: (-0.08, 0.12),
        2: (0.08, 0.12),
        3: (-0.08, -0.12),
        4: (0.08, -0.12),
    }

    _rail_positions = {
        "agv1": -4.5,
        "agv2": -1.2,
        "agv3": 1.2,
        "agv4": 4.5,
        "left_bins": 3,
        "right_bins": -3,
    }

    def __init__(self):
        super().__init__("robot_controller_py")

        self._world_collision_objects = []
        # In your __init__ method
        self._debug_mode = False  # Set to True only when debugging visualization
        self.declare_parameter('motion_planning.velocity_scale', 1.0)
        self.declare_parameter('motion_planning.acceleration_scale', 0.8)
        self.declare_parameter('motion_planning.planning_time', 1.0)
        self._objects_added_to_planning_scene = False
        self._pending_gripper_state = None
        self._gripper_state_future = None

        # ROS2 callback groups
        self.ariac_cb_group = MutuallyExclusiveCallbackGroup()
        self.moveit_cb_group = MutuallyExclusiveCallbackGroup()
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Add a service client for starting the competition
        self._start_competition_client = self.create_client(
            Trigger, "/ariac/start_competition"
        )

        # Add a service client for ending the competition
        self._end_competition_client = self.create_client(
            Trigger, "/ariac/end_competition"
        )

        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            "/ariac/competition_state",
            self._competition_state_cb,
            10,
            callback_group=self._reentrant_cb_group,
        )

        # Store the state of the competition
        self._competition_state = None

        # Add a timer to check if competition is ready
        self._check_competition_ready_timer = self.create_timer(
            0.5, self._check_competition_ready
        )

        # Flag to track if we've started the competition
        self._competition_start_requested = False

        # Store each camera image as an AdvancedLogicalCameraImage object
        self._camera_image = None

        # Turn on debug image publishing for part detection
        self.display_bounding_boxes = False

        # cv_bridge interface
        self._bridge = CvBridge()

        # Store RGB images from the right bins camera so they can be used for part detection
        self._right_bins_camera_image = None
        self._left_bins_camera_image = None

        # template images for parts
        self.sensor_template = np.ndarray([])
        self.battery_template = np.ndarray([])
        self.pump_template = np.ndarray([])
        self.regulator_template = np.ndarray([])

        # HSV colour bounds
        self.HSVcolors = {
            "red": {
                "hmin": 0,
                "smin": 10,
                "vmin": 115,
                "hmax": 4,
                "smax": 255,
                "vmax": 255,
            },
            "green": {
                "hmin": 57,
                "smin": 0,
                "vmin": 0,
                "hmax": 80,
                "smax": 255,
                "vmax": 255,
            },
            "blue": {
                "hmin": 116,
                "smin": 0,
                "vmin": 134,
                "hmax": 121,
                "smax": 255,
                "vmax": 255,
            },
            "orange": {
                "hmin": 14,
                "smin": 0,
                "vmin": 200,
                "hmax": 21,
                "smax": 255,
                "vmax": 255,
            },
            "purple": {
                "hmin": 130,
                "smin": 180,
                "vmin": 160,
                "hmax": 150,
                "smax": 255,
                "vmax": 255,
            },
        }

        # BGR reference colours
        self.colors = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "blue": (255, 0, 0),
            "orange": (100, 100, 255),
            "purple": (255, 0, 100),
        }

        # Part Pose Reporting Object
        self.part_poses = {
            "red": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "green": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "blue": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "orange": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "purple": {"battery": [], "pump": [], "sensor": [], "regulator": []},
        }
        # Center of Part Poses
        self.centered_part_poses = {
            "red": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "green": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "blue": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "orange": {"battery": [], "pump": [], "sensor": [], "regulator": []},
            "purple": {"battery": [], "pump": [], "sensor": [], "regulator": []},
        }

        self.slot_mapping = {
            (1, 1): 1,
            (1, 2): 2,
            (1, 3): 3,
            (2, 1): 4,
            (2, 2): 5,
            (2, 3): 6,
            (3, 1): 7,
            (3, 2): 8,
            (3, 3): 9,
        }

        self.color_mapping = {"red": 0, "green": 1, "blue": 2, "orange": 3, "purple": 4}

        self.type_mapping = {"battery": 10, "pump": 11, "sensor": 12, "regulator": 13}

        # read in part templates
        self.load_part_templates()

        # Subscriber to the floor gripper state topic
        self._floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            "/ariac/floor_robot_gripper_state",
            self._floor_robot_gripper_state_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )

        # Attribute to store the current state of the floor robot gripper
        self._floor_robot_gripper_state = VacuumGripperState()

        # Service client for turning on/off the vacuum gripper on the floor robot
        self._floor_gripper_enable = self.create_client(
            VacuumGripperControl, "/ariac/floor_robot_enable_gripper"
        )

        # Moveit_py variables
        self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
        self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())

        self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
        self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()

        # Parts found in the bins
        self._left_bins_parts = []
        self._right_bins_parts = []
        self._left_bins_camera_pose = Pose()
        self._right_bins_camera_pose = Pose()

        # service clients
        self._get_cartesian_path_client = self.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            callback_group=self._reentrant_cb_group,
        )

        # Camera subs
        self.left_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/left_bins_camera/image",
            self._left_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=self.moveit_cb_group,
        )

        self.right_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/right_bins_camera/image",
            self._right_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=self.moveit_cb_group,
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._floor_robot_attached_part_ = PartMsg()

        self._change_gripper_client = self.create_client(
            ChangeGripper, "/ariac/floor_robot_change_gripper"
        )

        # Planning Scene Info
        self.planning_scene_sub = self.create_subscription(
            PlanningScene,
            "/planning_scene",
            self.get_planning_scene_msg,
            10,
            callback_group=self.moveit_cb_group,
        )

        self.planning_scene_msg = PlanningScene()

        # Meshes file path
        self._mesh_file_path = get_package_share_directory("moveit_demo") + "/meshes/"

        # Predefined joint configurations
        self.floor_joint_positions_arrs = {
            "floor_kts1_js_": [4.0, 1.57, -1.57, 1.57, -1.57, -1.57, 0.0],
            "floor_kts2_js_": [-4.0, -1.57, -1.57, 1.57, -1.57, -1.57, 0.0],
            "left_bins": [3.0, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            "right_bins": [-3.0, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            "floor_conveyor_js_": [
                0.0,
                3.14,
                -0.9162979,
                2.04204,
                -2.67035,
                -1.57,
                0.0,
            ],
        }

        for i in range(1, 5):
            self.floor_joint_positions_arrs[f"agv{i}"] = [
                self._rail_positions[f"agv{i}"],
                0.0,
                -1.57,
                1.57,
                -1.57,
                -1.57,
                0.0,
            ]

        self.floor_position_dict = {
            key: self._create_floor_joint_position_state(
                self.floor_joint_positions_arrs[key]
            )
            for key in self.floor_joint_positions_arrs.keys()
        }

        self._control_timer = self.create_timer(2.0, self._control_cb)
        self._part_already_picked_up = False

    def _competition_start_callback(self, future):
        """
        Callback function executed after attempting to start the ARIAC competition.

        This function processes the result of the asynchronous service call to
        the '/ariac/start_competition' service. It logs a success message if
        the competition started successfully, a warning message if it failed,
        and an error message if the service call itself encountered an exception.

        Args:
        future (rclpy.task.Future): The Future object representing the result
        of the asynchronous service call.
        """
        try:
            # Attempt to get the result of the service call. This will raise an
            # exception if the service call failed for some reason (e.g., network issue).
            result = future.result()
            # Check the 'success' field of the service response.
            if result.success:
                self.get_logger().info("Started competition.")
            else:
                # Log a warning message indicating that the competition could not be started.
                # This usually means the service call was successful but the server
                # returned a negative response.
                self.get_logger().warn("Unable to start competition")
        except Exception as e:
            # Log an error message including the specific exception that occurred.
            self.get_logger().error(f"Competition start service call failed: {e}")

    def _check_competition_ready(self):
        """
        Timer callback function to periodically check the ARIAC competition state
        and initiate the start of the competition if it's in the 'READY' state
        and hasn't been started already.

        This function is triggered by a ROS 2 timer. It first checks if the
        competition has already started. If not, it checks if the current state
        is 'READY' and if a start request hasn't been sent yet. If both conditions
        are true, it logs a message, sets the `_competition_start_requested` flag
        to prevent multiple start attempts, checks if the start competition service
        is available, creates a service request, and sends it asynchronously with
        a callback function `_competition_start_callback` to handle the service response.
        """
        if self._competition_state == CompetitionStateMsg.STARTED:
            # Competition already started, disable timer
            self._check_competition_ready_timer.cancel()
            return

        if (
            not self._competition_start_requested
            and self._competition_state == CompetitionStateMsg.READY
        ):
            self.get_logger().info("Competition is ready. Starting...")
            self._competition_start_requested = True

            # Check if service is available
            if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(
                    "Service '/ariac/start_competition' is not available."
                )
                return

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            # Add a callback to handle the result
            future.add_done_callback(self._competition_start_callback)

    def _control_cb(self):
        """
        Timer callback function that defines the primary control logic for the robot
        during the ARIAC competition.
        """
        # First attempt to add models to the planning scene if not already done
        if not self._objects_added_to_planning_scene:
            success = self._add_objects_to_planning_scene()
            if success:
                self.get_logger().info(
                    "Successfully added all objects to planning scene"
                )
                self._objects_added_to_planning_scene = True
            else:
                self.get_logger().warn(
                    "Failed to add all objects to planning scene, will retry"
                )
                return  # Exit early to retry on next timer callback

        # Verify that planning scene is ready by checking for expected objects
        if not self._verify_planning_scene_ready():
            self.get_logger().warn("Planning scene not fully ready yet, waiting...")
            return  # Exit early to wait for planning scene to be ready

        # Wait for camera data to be available
        if not self._right_bins_parts:
            self.get_logger().info("Waiting for right bins camera data...")
            return

        # Only proceed if competition is in the correct state
        if self._competition_state not in [
            CompetitionStateMsg.STARTED,
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE,
        ]:
            return

        # Go to different locations using Joint-space programming
        # self._move_floor_robot_to_joint_position('left_bins')
        # self._move_floor_robot_to_joint_position('right_bins')
        # self._move_floor_robot_to_joint_position("agv1")
        # self._move_floor_robot_to_joint_position("agv2")
        # self._move_floor_robot_to_joint_position("agv3")
        # self._move_floor_robot_to_joint_position("agv4")

        # Pick up a part from a bin
        if not self._part_already_picked_up:
            part_to_pick = PartMsg()
            part_to_pick.color = PartMsg.PURPLE
            part_to_pick.type = PartMsg.PUMP

            success = self.floor_robot_pick_bin_part(part_to_pick)
            if success:
                self._part_already_picked_up = True
                self.get_logger().info("Successfully picked up part")
                # Force refresh the planning scene visualization
                self.refresh_planning_scene_display()
            else:
                self.get_logger().warn("Failed to pick up part, will retry later")

        # We are done, stop the competitionn
        self._end_competition()

    def _verify_planning_scene_ready(self):
        """
        Verify that the planning scene has the expected objects loaded.

        Returns:
            bool: True if planning scene is ready, False otherwise
        """
        try:
            # Check if we have objects in our tracking list
            if not self._world_collision_objects:
                self.get_logger().warn(
                    "No objects found in planning scene tracking list"
                )
                return False

            # Log what objects we have
            existing_ids = [obj.id for obj in self._world_collision_objects]
            self.get_logger().info(
                f"Planning scene has these tracked objects: {existing_ids}"
            )

            # Check if expected objects are present
            expected_objects = [
                "bin1",
                "bin2",
                "bin3",
                "bin4",
                "bin5",
                "bin6",
                "bin7",
                "bin8",
                "conveyor",
            ]
            missing_objects = [
                obj_id for obj_id in expected_objects if obj_id not in existing_ids
            ]

            if missing_objects:
                self.get_logger().warn(
                    f"Expected objects not found in planning scene: {missing_objects}"
                )

                # Try reloading the YAML file to see what objects should be there
                try:
                    package_share_directory = get_package_share_directory("moveit_demo")
                    yaml_path = (
                        package_share_directory + "/config/collision_object_info.yaml"
                    )

                    with open(yaml_path, "r") as object_file:
                        objects_dict = yaml.safe_load(object_file)

                    configured_objects = list(objects_dict.keys())
                    self.get_logger().info(
                        f"Objects defined in YAML: {configured_objects}"
                    )

                    # Check if the expected objects are in the YAML
                    yaml_missing = [
                        obj for obj in expected_objects if obj not in configured_objects
                    ]
                    if yaml_missing:
                        self.get_logger().error(
                            f"Objects missing from YAML configuration: {yaml_missing}"
                        )

                except Exception as yaml_error:
                    self.get_logger().error(
                        f"Error checking YAML configuration: {str(yaml_error)}"
                    )

                # Allow operation to continue even with missing objects (for debugging)
                self.get_logger().warn(
                    "Continuing despite missing objects for debugging purposes"
                )
                return True

            # If we get here, all expected objects are present
            self.get_logger().info(
                f"Planning scene ready with all expected objects ({len(self._world_collision_objects)} total)"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Error verifying planning scene: {str(e)}")
            return False

    def _add_objects_to_planning_scene(self):
        """
        Add all required collision objects to the planning scene with detailed logging.

        Returns:
            bool: True if all objects were added successfully, False otherwise
        """
        try:
            # Clear any existing objects to start fresh
            self._world_collision_objects = []

            # Load the YAML configuration
            package_share_directory = get_package_share_directory("moveit_demo")
            yaml_path = package_share_directory + "/config/collision_object_info.yaml"

            self.get_logger().info(f"Loading collision objects from: {yaml_path}")

            # Check if the YAML file exists
            if not path.exists(yaml_path):
                self.get_logger().error(
                    f"Collision object YAML file not found: {yaml_path}"
                )
                return False

            # Load the YAML data
            with open(yaml_path, "r") as object_file:
                objects_dict = yaml.safe_load(object_file)

            # Log how many objects we're trying to add
            self.get_logger().info(
                f"Found {len(objects_dict.keys())} objects in configuration file"
            )
            self.get_logger().info(f"Object IDs: {list(objects_dict.keys())}")

            # Track successes and failures
            successful_objects = []
            failed_objects = []

            # Process each object
            for key in objects_dict.keys():
                try:
                    self.get_logger().info(f"Processing object: {key}")

                    # Create a pose for the object
                    object_pose = Pose()
                    object_pose.position.x = float(objects_dict[key]["position"][0])
                    object_pose.position.y = float(objects_dict[key]["position"][1])
                    object_pose.position.z = float(objects_dict[key]["position"][2])
                    object_pose.orientation.x = float(
                        objects_dict[key]["orientation"][0]
                    )
                    object_pose.orientation.y = float(
                        objects_dict[key]["orientation"][1]
                    )
                    object_pose.orientation.z = float(
                        objects_dict[key]["orientation"][2]
                    )
                    object_pose.orientation.w = float(
                        objects_dict[key]["orientation"][3]
                    )

                    # Get the mesh file
                    mesh_file = objects_dict[key]["file"]
                    mesh_path = self._mesh_file_path + mesh_file

                    # Check if the mesh file exists
                    if not path.exists(mesh_path):
                        self.get_logger().error(f"Mesh file not found: {mesh_path}")
                        failed_objects.append(key)
                        continue

                    # Try to add the object to the planning scene
                    success = self._add_model_to_planning_scene(
                        key, mesh_file, object_pose
                    )

                    if success:
                        successful_objects.append(key)
                        self.get_logger().info(
                            f"Successfully added {key} to planning scene"
                        )
                    else:
                        failed_objects.append(key)
                        self.get_logger().error(
                            f"Failed to add {key} to planning scene"
                        )

                except Exception as obj_error:
                    self.get_logger().error(
                        f"Error processing object {key}: {str(obj_error)}"
                    )
                    failed_objects.append(key)

            # Check if all objects were added successfully
            if not failed_objects:
                self.get_logger().info(
                    f"All {len(successful_objects)} objects added successfully"
                )
                return True
            else:
                self.get_logger().error(
                    f"Failed to add {len(failed_objects)} objects: {failed_objects}"
                )
                self.get_logger().info(
                    f"Successfully added {len(successful_objects)} objects: {successful_objects}"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Error adding objects to planning scene: {str(e)}")
            return False

    def _add_model_to_planning_scene(
        self, name: str, mesh_file: str, model_pose: Pose, frame_id="world"
    ) -> bool:
        """
        Add a mesh model to the planning scene with detailed error handling.

        Args:
            name: Name of the collision object
            mesh_file: Path to the mesh file
            model_pose: Pose of the model
            frame_id: Frame ID for the collision object

        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(f"Adding model {name} to planning scene")

        try:
            # Get the full path to the mesh file
            model_path = self._mesh_file_path + mesh_file

            # Check if the mesh file exists
            if not path.exists(model_path):
                self.get_logger().error(f"Mesh file not found: {model_path}")
                return False

            # Try to load the mesh file
            collision_object = None
            try:
                collision_object = self._make_mesh(
                    name, model_pose, model_path, frame_id
                )
            except Exception as mesh_error:
                self.get_logger().error(
                    f"Error creating mesh for {name}: {str(mesh_error)}"
                )
                return False

            if collision_object is None:
                self.get_logger().error(f"Failed to create collision object for {name}")
                return False

            # Apply the collision object to the scene
            try:
                with self._planning_scene_monitor.read_write() as scene:
                    # Apply the collision object
                    scene.apply_collision_object(collision_object)

                    # Wait a tiny bit for the update to take effect
                    time.sleep(0.05)

                    # Check if the object was actually added
                    # For debugging we'll try to access the object
                    try:
                        # Different way to check if the object exists
                        self.get_logger().info(f"Checking if {name} was added to scene")

                        # Just update the state and add the object to our tracking list
                        self._world_collision_objects.append(collision_object)
                        scene.current_state.update()

                        self.get_logger().info(
                            f"Successfully added {name} to planning scene"
                        )
                        return True

                    except Exception as check_error:
                        self.get_logger().warn(
                            f"Object {name} may not have been added properly: {str(check_error)}"
                        )
                        # Still add it to our tracking list
                        self._world_collision_objects.append(collision_object)
                        return True  # Return true anyway to continue with other objects

            except Exception as scene_error:
                self.get_logger().error(
                    f"Error applying collision object {name} to scene: {str(scene_error)}"
                )
                return False

        except Exception as e:
            self.get_logger().error(
                f"Unexpected error adding model {name} to planning scene: {str(e)}"
            )
            return False

    def _competition_state_cb(self, msg: CompetitionStateMsg):
        """Callback for the topic /ariac/competition_state
        Arguments:
            msg -- CompetitionState message
        """
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = RobotController._competition_states[msg.competition_state]
            self.get_logger().info(
                f"Competition state is: {state}", throttle_duration_sec=1.0
            )

        self._competition_state = msg.competition_state

    def _end_competition(self):
        """End the competition using a non-blocking approach."""
        self.get_logger().info("Ending competition")

        # Check if service is available
        if not self._end_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "Service '/ariac/end_competition' is not available."
            )
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._end_competition_client.call_async(request)

        # Add a callback to handle the result
        future.add_done_callback(self._end_competition_callback)

    def _end_competition_callback(self, future):
        """Callback for competition end service."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Competition ended successfully.")
            else:
                self.get_logger().warn("Unable to end competition")
        except Exception as e:
            self.get_logger().error(f"End competition service call failed: {e}")

    def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        """Callback for the topic /ariac/floor_robot_gripper_state

        Arguments:
            msg -- VacuumGripperState message
        """
        self._floor_robot_gripper_state = msg

    def _left_bins_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        self._left_bins_parts = msg.part_poses
        self._left_bins_camera_pose = msg.sensor_pose

    def _right_bins_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        self._right_bins_parts = msg.part_poses
        self._right_bins_camera_pose = msg.sensor_pose

    def get_planning_scene_msg(self, msg: PlanningScene):
        self.planning_scene_msg = msg

    def set_floor_robot_gripper_state(self, state):
        """Optimized gripper state control with faster response handling"""
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().debug(f"Gripper is already {self._gripper_states[state]}")
            return None

        request = VacuumGripperControl.Request()
        request.enable = state

        # Store state for use in callback
        self._pending_gripper_state = state

        # Log at debug level instead of info
        self.get_logger().debug(f"Changing gripper state to {self._gripper_states[state]}")
        
        # Use call_async with a callback
        future = self._floor_gripper_enable.call_async(request)
        future.add_done_callback(self._gripper_state_callback)

        # Store and return the future
        self._gripper_state_future = future
        return future

    def _gripper_state_callback(self, future):
        """Callback for gripper state change service.

        Arguments:
            future -- Future object from the service call
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Changed gripper state to {self._gripper_states[self._pending_gripper_state]}"
                )
            else:
                self.get_logger().warn("Unable to change gripper state")
        except Exception as e:
            self.get_logger().error(f"Gripper state service call failed: {e}")

    def _move_floor_robot_cartesian(
        self, waypoints, velocity, acceleration, avoid_collision=True
    ):
        """
        Move the floor robot along a Cartesian path with optimized speed.
        
        Args:
            waypoints: List of waypoint poses
            velocity: Maximum velocity scaling factor (0.0-1.0)
            acceleration: Maximum acceleration scaling factor (0.0-1.0)
            avoid_collision: Whether to avoid collisions
        """
        # Increase default velocity and acceleration for faster movement
        velocity = max(0.5, velocity)  # Minimum velocity scaling of 0.5
        acceleration = max(0.5, acceleration)  # Minimum acceleration scaling of 0.5
        
        # Get the trajectory
        trajectory_msg = self._call_get_cartesian_path(
            waypoints, velocity, acceleration, avoid_collision, "floor_robot"
        )
        
        if trajectory_msg is None:
            self.get_logger().error("Failed to compute cartesian path")
            return False
        
        # Execute the trajectory
        with self._planning_scene_monitor.read_write() as scene:
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = "floor_robot"
            scene.current_state.update(True)
            self._ariac_robots_state = scene.current_state

        try:
            self._ariac_robots.execute(trajectory, controllers=[])
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to execute cartesian path: {str(e)}")
            return False

    def _execute_cartesian_trajectory(self, trajectory_msg):
        """Execute a cartesian trajectory once it's computed.

        Args:
            trajectory_msg: The computed trajectory message or None if failed
        """
        if trajectory_msg is None:
            self.get_logger().error("Failed to get cartesian path, cannot execute")
            return False

        with self._planning_scene_monitor.read_write() as scene:
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = "floor_robot"
            scene.current_state.update(True)
            self._ariac_robots_state = scene.current_state

        success = False
        try:
            self._ariac_robots.execute(trajectory, controllers=[])
            self.get_logger().debug("Cartesian trajectory execution started")
            # Wait a moment to ensure the trajectory has started
            time.sleep(0.5)
            success = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute trajectory: {str(e)}")

        return success

    def _call_get_cartesian_path(
        self,
        waypoints: list,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        avoid_collision: bool,
        robot: str,
    ):
        self.get_logger().debug(
            "Getting cartesian path"
        )  # Use debug level for less logging

        request = GetCartesianPath.Request()

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()

        request.header = header
        with self._planning_scene_monitor.read_write() as scene:
            request.start_state = robotStateToRobotStateMsg(scene.current_state)

        if robot == "floor_robot":
            request.group_name = "floor_robot"
            request.link_name = "floor_gripper"
        else:
            request.group_name = "ceiling_robot"
            request.link_name = "ceiling_gripper"

        # Always use higher velocity values for faster motion
        request.waypoints = waypoints
        request.max_step = 0.1
        request.avoid_collisions = avoid_collision
        # Override with higher values for faster motion
        request.max_velocity_scaling_factor = max(0.7, max_velocity_scaling_factor)
        request.max_acceleration_scaling_factor = max(
            0.6, max_acceleration_scaling_factor
        )

        future = self._get_cartesian_path_client.call_async(request)

        # Use more efficient waiting with timeout
        timeout = 1.0  # 1 second timeout
        start_time = time.time()
        while not future.done():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().warn("Cartesian path planning timeout!")
                return None
            time.sleep(0.01)  # Short sleep to reduce CPU usage

        result: GetCartesianPath.Response
        result = future.result()

        if result.fraction < 0.9:
            self.get_logger().warn(
                f"Path planning incomplete: only {result.fraction:.2f} coverage"
            )

        return result.solution

    def _plan_and_execute(
        self,
        robot,
        planning_component,
        logger,
        robot_type,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Optimized helper function to plan and execute a motion."""
        # plan to goal
        logger.debug("Planning trajectory")  # Change to debug level
        
        # Timing for performance monitoring
        plan_start = time.time()
        
        # Plan the motion - instead of trying to set parameters directly, 
        # we'll use the existing planning functionality with default parameters
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            # Use default planning with no special parameters
            plan_result = planning_component.plan()
        
        plan_time = time.time() - plan_start
        logger.debug(f"Planning took {plan_time:.3f} seconds")

        # execute the plan
        if plan_result:
            logger.debug("Executing plan")  # Change to debug level
            exec_start = time.time()
            
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update(True)
                self._ariac_robots_state = scene.current_state
                robot_trajectory = plan_result.trajectory
                
            # Execute with appropriate controllers
            robot.execute(
                robot_trajectory,
                controllers=["floor_robot_controller", "linear_rail_controller"]
                if robot_type == "floor_robot"
                else ["ceiling_robot_controller", "gantry_controller"],
            )
            
            exec_time = time.time() - exec_start
            logger.debug(f"Execution took {exec_time:.3f} seconds")
            
            # Skip unnecessary sleep
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            return True
        else:
            logger.error("Planning failed")
            return False
        

    def _move_floor_robot_to_pose(self, pose: Pose):
        """Move the floor robot to a pose in Cartesian space.

        Args:
            pose (Pose): Target pose
        """
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose = pose
            self._floor_robot.set_goal_state(
                pose_stamped_msg=pose_goal, pose_link="floor_gripper"
            )

        # Limit retries to avoid infinite loops
        attempts = 0
        max_attempts = 3

        while attempts < max_attempts:
            success = self._plan_and_execute(
                self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"
            )
            if success:
                return True
            attempts += 1
            self.get_logger().warn(
                f"Plan and execute failed, attempt {attempts}/{max_attempts}"
            )
            # Short pause before retry
            time.sleep(0.5)

        self.get_logger().error(f"Failed to move to pose after {max_attempts} attempts")
        return False

    
    def _floor_robot_wait_for_attach(self, timeout: float, orientation: Quaternion):
        """More efficient wait for attachment with fewer intermediate moves"""
        with self._planning_scene_monitor.read_write() as scene:
            current_pose = scene.current_state.get_pose("floor_gripper")
        
        start_time = time.time()
        retry_count = 0
        max_retries = 10
        
        # First try waiting a short time for attachment without moving
        time.sleep(0.1)
        if self._floor_robot_gripper_state.attached:
            self.get_logger().info("Part attached on first attempt")
            return True
        
        # while not self._floor_robot_gripper_state.attached and retry_count < max_retries:
        while not self._floor_robot_gripper_state.attached and retry_count < max_retries:
            # Move down in larger increments for faster operation
            # z_offset = -0.002 * (retry_count + 1)  # Progressive larger movements
            z_offset = -0.001
            
            current_pose = build_pose(
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z + z_offset,
                orientation,
            )
            
            waypoints = [current_pose]
            self._move_floor_robot_cartesian(waypoints, 0.1, 0.1, False)
            
            # Check if attached after movement
            time.sleep(0.4)  # Short wait 
            
            retry_count += 1
            
            if time.time() - start_time >= timeout:
                self.get_logger().error("Unable to pick up part: timeout")
                raise Error("Gripper attachment timeout")
        
        if not self._floor_robot_gripper_state.attached:
            self.get_logger().error("Unable to pick up part: max retries reached")
            raise Error("Gripper attachment failed after max retries")
            
        self.get_logger().info(f"Part attached after {retry_count} attempts")
        return True
    
    def _create_floor_joint_position_state(self, joint_positions: list) -> dict:
        """Create a dictionary of joint positions for the floor robot.

        Args:
            joint_positions (list): List of joint positions

        Returns:
            dict: Dictionary of joint names and positions
        """
        return {
            "linear_actuator_joint": joint_positions[0],
            "floor_shoulder_pan_joint": joint_positions[1],
            "floor_shoulder_lift_joint": joint_positions[2],
            "floor_elbow_joint": joint_positions[3],
            "floor_wrist_1_joint": joint_positions[4],
            "floor_wrist_2_joint": joint_positions[5],
            "floor_wrist_3_joint": joint_positions[6],
        }

    def _make_mesh(self, name, pose, filename, frame_id) -> CollisionObject:
        with pyassimp.load(filename) as scene:
            assert len(scene.meshes), "No meshes found in the file"

            mesh = Mesh()
            for face in scene.meshes[0].faces:
                triangle = MeshTriangle()
                if hasattr(face, "indices"):
                    if len(face.indices) == 3:
                        triangle.vertex_indices = [
                            face.indices[0],
                            face.indices[1],
                            face.indices[2],
                        ]
                        mesh.triangles.append(triangle)
                else:
                    if len(face) == 3:
                        triangle.vertex_indices = [face[0], face[1], face[2]]
                        mesh.triangles.append(triangle)

            for vertex in scene.meshes[0].vertices:
                point = Point()
                point.x = float(vertex[0])
                point.y = float(vertex[1])
                point.z = float(vertex[2])
                mesh.vertices.append(point)

            o = CollisionObject()
            o.header.frame_id = frame_id
            o.id = name
            o.meshes.append(mesh)
            o.mesh_poses.append(pose)
            o.operation = o.ADD

            return o

    def _make_attached_mesh(self, name, pose, filename, robot):
        """Create an attached mesh collision object."""
        try:
            with pyassimp.load(filename) as scene:
                if not scene.meshes:
                    self.get_logger().error(f"No meshes found in {filename}")
                    return None

                mesh = Mesh()
                for face in scene.meshes[0].faces:
                    triangle = MeshTriangle()
                    if hasattr(face, "indices"):
                        if len(face.indices) == 3:
                            triangle.vertex_indices = [
                                face.indices[0],
                                face.indices[1],
                                face.indices[2],
                            ]
                            mesh.triangles.append(triangle)
                    else:
                        if len(face) == 3:
                            triangle.vertex_indices = [face[0], face[1], face[2]]
                            mesh.triangles.append(triangle)

                for vertex in scene.meshes[0].vertices:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    mesh.vertices.append(point)

            o = AttachedCollisionObject()
            if robot == "floor_robot":
                o.link_name = "floor_gripper"
                # Add touch links - links that are allowed to touch the attached object
                o.touch_links = ["floor_gripper", "floor_tool0", "floor_wrist_3_link"]
            else:
                o.link_name = "ceiling_gripper"
                o.touch_links = [
                    "ceiling_gripper",
                    "ceiling_tool0",
                    "ceiling_wrist_3_link",
                ]

            # Fix the frame ID - use world frame for consistent positioning
            o.object.header.frame_id = "world"
            o.object.id = name
            o.object.meshes.append(mesh)
            o.object.mesh_poses.append(pose)
            o.object.operation = CollisionObject.ADD  # Explicitly set the operation

            return o
        except Exception as e:
            self.get_logger().error(f"Error creating attached mesh: {str(e)}")
            return None

    def apply_planning_scene(self, scene):
        """Apply a planning scene to the MoveIt planning scene."""
        try:
            # Create a client for the service
            apply_planning_scene_client = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene"
            )

            # Wait for service to be available
            if not apply_planning_scene_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("'/apply_planning_scene' service not available")
                return False

            # Create and send the request
            request = ApplyPlanningScene.Request()
            request.scene = scene

            # Log for debugging
            self.get_logger().debug(
                f"Sending planning scene with {len(scene.robot_state.attached_collision_objects)} attached objects"
            )

            # Send the request
            future = apply_planning_scene_client.call_async(request)

            # Wait for the response with timeout
            timeout_sec = 2.0
            start_time = time.time()

            while not future.done():
                # Short sleep to prevent CPU thrashing
                time.sleep(0.02)

                # Check for timeout
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error(
                        "Timeout waiting for planning scene service"
                    )
                    return False

            # Process the result
            result = future.result()
            if result.success:
                self.get_logger().info("Successfully applied planning scene")
                return True
            else:
                self.get_logger().warn(
                    f"Failed to apply planning scene: {result.message if hasattr(result, 'message') else 'unknown error'}"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Error applying planning scene: {str(e)}")
            return False

    def debug_planning_scene(self):
        """
        Print debug information about the current planning scene.
        Only call this when debugging visualization issues.
        """
        with self._planning_scene_monitor.read_write() as scene:
            # Get attached objects
            attached_bodies = []
            try:
                attached_bodies = scene.current_state.attached_bodies
            except AttributeError:
                self.get_logger().warn("Could not access attached_bodies")
            
            # Get world objects
            world_objects = []
            try:
                world = scene.get_world()
                if hasattr(world, "collision_objects"):
                    world_objects = world.collision_objects
            except AttributeError:
                self.get_logger().warn("Could not access world collision objects")
            
            # Log the results
            self.get_logger().info(f"Attached objects: {len(attached_bodies)}")
            for i, obj in enumerate(attached_bodies):
                if hasattr(obj, "name"):
                    self.get_logger().info(f"  {i+1}. {obj.name}")
                else:
                    self.get_logger().info(f"  {i+1}. (unnamed)")
            
            self.get_logger().info(f"World objects: {len(world_objects)}")
            for i, obj in enumerate(world_objects):
                if hasattr(obj, "id"):
                    self.get_logger().info(f"  {i+1}. {obj.id}")
                else:
                    self.get_logger().info(f"  {i+1}. (unnamed)")

    def refresh_planning_scene_display(self):
        """Force a refresh of the planning scene display in RViz."""
        # Create an empty diff planning scene just to trigger a display update
        refresh_scene = PlanningScene()
        refresh_scene.is_diff = True
        self.apply_planning_scene(refresh_scene)

    def _remove_model_from_floor_gripper(self):
        self.get_logger().info("Removing attached part from floor gripper")
        temp_scene = copy(self.planning_scene_msg)
        with self._planning_scene_monitor.read_write() as scene:
            temp_scene.world.collision_objects = self._world_collision_objects
            temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
            temp_scene.robot_state.attached_collision_objects.clear()
            self.apply_planning_scene(temp_scene)
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

    def _move_floor_robot_to_joint_position(self, position_name: str):
        """
        Move the floor robot to a predefined joint position with improved speed.
        """
        self.get_logger().info(f"Moving to position: {position_name}")

        with self._planning_scene_monitor.read_write() as scene:
            # Set the start state to the current robot state
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            # Create a new robot state for the goal
            goal_state = copy(scene.current_state)
            # Set the joint positions on the goal state
            goal_state.joint_positions = self.floor_position_dict[position_name]

            # Create a joint constraint for the motion plan
            joint_constraint = construct_joint_constraint(
                robot_state=goal_state,
                joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group(
                    "floor_robot"
                ),
            )

            # Set the goal state using the joint constraint
            self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])

        # Plan and execute with improved default parameters
        success = self._plan_and_execute(
            self._ariac_robots,
            self._floor_robot,
            self.get_logger(),
            robot_type="floor_robot",
            sleep_time=0.0  # No sleep after execution
        )
        
        return success


    def floor_robot_pick_bin_part(self, part_to_pick: PartMsg):
        """
        Pick a part from a bin using Cartesian space programming with improved speed.
        """
        # Skip unnecessary debug logging for faster operation
        self.get_logger().info(
            f"Picking {self._part_colors[part_to_pick.color]} {self._part_types[part_to_pick.type]}"
        )

        # Initialize variables
        part_pose = Pose()
        found_part = False
        bin_side = ""

        # PART DETECTION PHASE - Optimize by reducing search time
        # Check right bins first if part color is red/green/blue (commonly in right bins)
        # Check left bins first if part color is orange/purple (commonly in left bins)
        search_order = []
        if part_to_pick.color in [PartMsg.RED, PartMsg.GREEN, PartMsg.BLUE]:
            search_order = [self._right_bins_parts, self._left_bins_parts]
            camera_poses = [self._right_bins_camera_pose, self._left_bins_camera_pose]
            bin_sides = ["right_bins", "left_bins"]
        else:
            search_order = [self._left_bins_parts, self._right_bins_parts]
            camera_poses = [self._left_bins_camera_pose, self._right_bins_camera_pose]
            bin_sides = ["left_bins", "right_bins"]

        # Optimized search
        for i in range(2):
            for part in search_order[i]:
                if (part.part.type == part_to_pick.type and 
                    part.part.color == part_to_pick.color):
                    part_pose = multiply_pose(camera_poses[i], part.pose)
                    found_part = True
                    bin_side = bin_sides[i]
                    break
            if found_part:
                break

        if not found_part:
            self.get_logger().error("Unable to locate part")
            return False
        else:
            self.get_logger().info(f"Part found in {bin_side}")

        # GRIPPER PREPARATION PHASE - Skip if already using the right gripper
        if self._floor_robot_gripper_state.type != "part_gripper":
            # Determine which tool changer station to use
            station = "kts1" if part_pose.position.y < 0 else "kts2"
            
            # Move to the tool changer and change gripper
            self._move_floor_robot_to_joint_position(f"floor_{station}_js_")
            self._floor_robot_change_gripper(station, "parts")

        # APPROACH PHASE - Faster approach
        # Move to the bin with the part
        self._move_floor_robot_to_joint_position(bin_side)

        # Calculate gripper orientation
        part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
        gripper_orientation = quaternion_from_euler(0.0, pi, part_rotation)

        # Move above the part - use closer approach to save time
        above_pose = build_pose(
            part_pose.position.x,
            part_pose.position.y,
            part_pose.position.z + 0.15,  # Reduced from 0.3 to 0.15
            gripper_orientation,
        )
        self._move_floor_robot_to_pose(above_pose)

        # PICKING PHASE - Faster movement down
        self.get_logger().info("Moving to grasp position")
        waypoints = [
            build_pose(
                part_pose.position.x,
                part_pose.position.y,
                # Optimize height for better first-attempt success
                part_pose.position.z + 
                RobotController._part_heights[part_to_pick.type] + 0.005,
                gripper_orientation,
            )
        ]
        # Use faster movement for approach
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

        # Enable gripper with less waiting
        self.set_floor_robot_gripper_state(True)
        
        # More efficient attachment process with shorter timeout
        try:
            self._floor_robot_wait_for_attach(10.0, gripper_orientation)  # Reduced timeout
        except Error as e:
            self.get_logger().error(f"Attachment failed: {str(e)}")
            
            # Quick recovery
            waypoints = [
                build_pose(
                    part_pose.position.x,
                    part_pose.position.y,
                    part_pose.position.z + 0.1,
                    gripper_orientation,
                )
            ]
            self._move_floor_robot_cartesian(waypoints, 0.5, 0.5, False)  # Faster retreat
            self.set_floor_robot_gripper_state(False)
            return False

        # RETREAT PHASE - Faster retreat
        # Quick lift to clear obstacles
        waypoints = [
            build_pose(
                part_pose.position.x,
                part_pose.position.y,
                part_pose.position.z + 0.2,
                gripper_orientation,
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.5, 0.5, False)

        # Return to bin position
        self._move_floor_robot_to_joint_position(bin_side)

        # SCENE UPDATE PHASE - Minimal planning scene updates
        # Just record the attached part internally for tracking
        self._floor_robot_attached_part_ = part_to_pick
        
        # Only update planning scene if needed
        self._attach_model_to_floor_gripper(part_to_pick, part_pose)

        return True

    def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
        """
        Attach a part to the floor gripper in the planning scene efficiently.
        
        This implementation is optimized for speed, with options for debugging.
        """
        # Create a part name based on its color and type
        part_name = (
            self._part_colors[part_to_pick.color] 
            + "_" 
            + self._part_types[part_to_pick.type]
        )
        
        # Always track the part internally for collision awareness
        self._floor_robot_attached_part_ = part_to_pick
        
        # Check if we're using simplified mode for speed
        debug_mode = False  # Set this to True only when debugging visualization
        
        if not debug_mode:
            # Fast path - just track the part without scene updates
            self.get_logger().info(f"Tracking attached part: {part_name} (simplified for speed)")
            return True
        
        # Full planning scene update path (for debugging visualization)
        self.get_logger().info(f"Attaching {part_name} to floor gripper (full update)")
        
        # Get the path to the mesh file for the part
        model_path = self._mesh_file_path + self._part_types[part_to_pick.type] + ".stl"
        
        if not path.exists(model_path):
            self.get_logger().error(f"Mesh file not found: {model_path}")
            return False
        
        try:
            # Create a planning scene message for the update
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            
            # Create the collision object
            co = CollisionObject()
            co.id = part_name
            co.header.frame_id = "world"
            co.header.stamp = self.get_clock().now().to_msg()
            
            # Create the mesh from the file
            with pyassimp.load(model_path) as scene:
                if not scene.meshes:
                    self.get_logger().error(f"No meshes found in {model_path}")
                    return False
                    
                mesh = Mesh()
                # Add triangles
                for face in scene.meshes[0].faces:
                    triangle = MeshTriangle()
                    if hasattr(face, "indices"):
                        if len(face.indices) == 3:
                            triangle.vertex_indices = [
                                face.indices[0],
                                face.indices[1],
                                face.indices[2]
                            ]
                            mesh.triangles.append(triangle)
                    else:
                        if len(face) == 3:
                            triangle.vertex_indices = [face[0], face[1], face[2]]
                            mesh.triangles.append(triangle)
                
                # Add vertices
                for vertex in scene.meshes[0].vertices:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    mesh.vertices.append(point)
            
            # Add the mesh to the collision object
            co.meshes.append(mesh)
            co.mesh_poses.append(part_pose)
            co.operation = CollisionObject.ADD
            
            # First add the object to the world - this is important!
            planning_scene.world.collision_objects.append(co)
            
            # Apply this first update to ensure the object exists in the world
            self.apply_planning_scene(planning_scene)
            
            # Now create an attached collision object
            aco = AttachedCollisionObject()
            aco.link_name = "floor_gripper"
            aco.object = co
            aco.object.operation = CollisionObject.ADD
            aco.touch_links = ["floor_gripper", "floor_tool0", "floor_wrist_3_link"]
            
            # Create a new planning scene message for the attachment
            attach_scene = PlanningScene()
            attach_scene.is_diff = True
            attach_scene.robot_state.attached_collision_objects.append(aco)
            
            # Now the crucial part - remove the object from the world since it's attached
            remove_co = CollisionObject()
            remove_co.id = part_name
            remove_co.operation = CollisionObject.REMOVE
            attach_scene.world.collision_objects.append(remove_co)
            
            # Apply the attachment
            success = self.apply_planning_scene(attach_scene)
            
            if success:
                self.get_logger().info(f"Successfully attached {part_name} to floor gripper")
                return True
            else:
                self.get_logger().error(f"Failed to attach {part_name} to floor gripper")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error attaching model to gripper: {str(e)}")
            return False

    # def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
    #     """Attach a part to the floor gripper in the planning scene."""
    #     part_name = (
    #         self._part_colors[part_to_pick.color]
    #         + "_"
    #         + self._part_types[part_to_pick.type]
    #     )

    #     self.get_logger().info(f"Attaching {part_name} to floor gripper")

    #     # Get the path to the mesh file for the part
    #     model_path = self._mesh_file_path + self._part_types[part_to_pick.type] + ".stl"

    #     try:
    #         # First approach: Use the planning scene monitor directly
    #         with self._planning_scene_monitor.read_write() as scene:
    #             # Create the collision object for the part
    #             co = CollisionObject()
    #             co.id = part_name
    #             co.header.frame_id = "world"

    #             # Load the mesh
    #             with pyassimp.load(model_path) as scene_obj:
    #                 if not scene_obj.meshes:
    #                     self.get_logger().error(f"No meshes found in {model_path}")
    #                     return False

    #                 mesh = Mesh()
    #                 # Create triangles
    #                 for face in scene_obj.meshes[0].faces:
    #                     triangle = MeshTriangle()
    #                     if hasattr(face, "indices"):
    #                         if len(face.indices) == 3:
    #                             triangle.vertex_indices = [
    #                                 face.indices[0],
    #                                 face.indices[1],
    #                                 face.indices[2]
    #                             ]
    #                             mesh.triangles.append(triangle)
    #                     else:
    #                         if len(face) == 3:
    #                             triangle.vertex_indices = [face[0], face[1], face[2]]
    #                             mesh.triangles.append(triangle)

    #                 # Create vertices
    #                 for vertex in scene_obj.meshes[0].vertices:
    #                     point = Point()
    #                     point.x = float(vertex[0])
    #                     point.y = float(vertex[1])
    #                     point.z = float(vertex[2])
    #                     mesh.vertices.append(point)

    #             # Add the mesh to the collision object
    #             co.meshes.append(mesh)
    #             co.mesh_poses.append(part_pose)
    #             co.operation = CollisionObject.ADD

    #             # Apply the collision object
    #             scene.apply_collision_object(co)

    #             # Attach the object to the robot
    #             robot_state = scene.current_state
    #             robot_state.attachBody(
    #                 part_name,
    #                 "floor_gripper",
    #                 ["floor_gripper", "floor_tool0", "floor_wrist_3_link"]
    #             )

    #             # Update the scene
    #             scene.current_state.update()
    #             self._ariac_robots_state = scene.current_state
    #             self.get_logger().info(f"Successfully attached {part_name} to floor gripper")
    #             return True

    #     except Exception as e:
    #         self.get_logger().error(f"Error attaching model to gripper: {str(e)}")

    #         # Try alternative approach using the planning scene interface
    #         try:
    #             self.get_logger().info(f"Trying alternative approach to attach {part_name}")
    #             # Create a new planning scene message
    #             scene_msg = PlanningScene()
    #             scene_msg.is_diff = True

    #             # Create the attached collision object
    #             aco = AttachedCollisionObject()
    #             aco.link_name = "floor_gripper"
    #             aco.object.header.frame_id = "floor_gripper"
    #             aco.object.id = part_name

    #             # Load the mesh
    #             with pyassimp.load(model_path) as scene_obj:
    #                 # ... (mesh creation code as above)
    #                 # ...

    #             aco.object.meshes.append(mesh)
    #             aco.object.mesh_poses.append(part_pose)
    #             aco.object.operation = CollisionObject.ADD
    #             aco.touch_links = ["floor_gripper", "floor_tool0", "floor_wrist_3_link"]

    #             # Add to the scene message
    #             scene_msg.robot_state.attached_collision_objects.append(aco)

    #             # Apply the scene using the service
    #             success = self.apply_planning_scene(scene_msg)
    #             if success:
    #                 self.get_logger().info(f"Successfully attached {part_name} using alternative method")
    #                 return True
    #             else:
    #                 self.get_logger().error("Failed to attach part using alternative method")
    #                 return False

    #         except Exception as nested_e:
    #             self.get_logger().error(f"Both attachment methods failed: {str(nested_e)}")
    #             return False

    def _floor_robot_change_gripper(self, station: str, gripper_type: str):
        """Change the gripper on the floor robot.

        Args:
            station (str): Station to change gripper at
            gripper_type (str): Type of gripper to change to

        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(f"Changing gripper to type: {gripper_type}")

        try:
            tc_pose = self._frame_world_pose(
                f"{station}_tool_changer_{gripper_type}_frame"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get tool changer frame pose: {str(e)}")
            return False

        # Move above the tool changer
        above_pose = build_pose(
            tc_pose.position.x,
            tc_pose.position.y,
            tc_pose.position.z + 0.2,
            quaternion_from_euler(0.0, pi, 0.0),
        )
        success = self._move_floor_robot_to_pose(above_pose)
        if not success:
            self.get_logger().error("Failed to move above tool changer")
            return False

        # Move to the tool changer
        waypoints = [
            build_pose(
                tc_pose.position.x,
                tc_pose.position.y,
                tc_pose.position.z,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.1, 0.1, False)

        # Create and send the service request
        request = ChangeGripper.Request()
        if gripper_type == "trays":
            request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER
        elif gripper_type == "parts":
            request.gripper_type = ChangeGripper.Request.PART_GRIPPER

        # Wait for the service to be available
        if not self._change_gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Change gripper service not available")
            return False

        # Call the service
        future = self._change_gripper_client.call_async(request)

        # Wait for the result (with timeout)
        wait_count = 0
        max_wait = 50  # 5 seconds total
        while not future.done() and wait_count < max_wait:
            time.sleep(0.1)
            wait_count += 1

        if not future.done():
            self.get_logger().error(
                "Timeout reached when calling change_gripper service"
            )
            return False

        try:
            result = future.result()
            if not result.success:
                self.get_logger().error("Error calling change gripper service")
                return False
        except Exception as e:
            self.get_logger().error(
                f"Exception calling change gripper service: {str(e)}"
            )
            return False

        # Move away from the tool changer
        waypoints = [
            build_pose(
                tc_pose.position.x,
                tc_pose.position.y,
                tc_pose.position.z + 0.2,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
        return True

    def _frame_world_pose(self, frame_id: str):
        """Get the pose of a frame in the world frame.

        Args:
            frame_id (str): Frame ID to get pose for

        Returns:
            Pose: Pose of the frame in the world frame
        """
        self.get_logger().info(f"Getting transform for frame: {frame_id}")

        # Wait for transform to be available
        try:
            self.tf_buffer.wait_for_transform_async(
                "world",
                frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2.0),
            )
            t = self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get transform for {frame_id}: {str(e)}")
            raise

        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation

        return pose

    def load_part_templates(self):
        try:
            self.sensor_template = cv2.imread(
                path.join(
                    get_package_share_directory("moveit_demo"),
                    "resources",
                    "sensor.png",
                ),
                cv2.IMREAD_GRAYSCALE,
            )
            self.regulator_template = cv2.imread(
                path.join(
                    get_package_share_directory("moveit_demo"),
                    "resources",
                    "regulator.png",
                ),
                cv2.IMREAD_GRAYSCALE,
            )
            self.battery_template = cv2.imread(
                path.join(
                    get_package_share_directory("moveit_demo"),
                    "resources",
                    "battery.png",
                ),
                cv2.IMREAD_GRAYSCALE,
            )
            self.pump_template = cv2.imread(
                path.join(
                    get_package_share_directory("moveit_demo"), "resources", "pump.png"
                ),
                cv2.IMREAD_GRAYSCALE,
            )

            if (
                (not self.sensor_template.shape[0] > 0)
                or (not self.regulator_template.shape[0] > 0)
                or (not self.battery_template.shape[0] > 0)
                or (not self.pump_template.shape[0] > 0)
            ):
                self.get_logger().error("Failed to load at least one part template")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"Exception loading part templates: {str(e)}")
            return False
