from math import cos, sin, pi
from copy import copy
import time
import PyKDL
from moveit_demo.fancy_log import FancyLog
from sympy import Quaternion
from ament_index_python import get_package_share_directory
from moveit import MoveItPy, PlanningSceneMonitor
import rclpy
import pyassimp
import yaml
import random
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterEvent

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
    ConveyorParts as ConveyorPartsMsg,
    Order as OrderMsg,
)

from ariac_msgs.srv import (
    VacuumGripperControl,
    ChangeGripper,
    MoveAGV,
    PerformQualityCheck,
    SubmitOrder,
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

    _rail_positions = {
        "agv1": -4.5,
        "agv2": -1.2,
        "agv3": 1.2,
        "agv4": 4.5,
        "left_bins": 3,
        "right_bins": -3,
    }

    _agv_destinations = {
        MoveAGV.Request.KITTING: "kitting",
        MoveAGV.Request.ASSEMBLY_FRONT: "assembly_front",
        MoveAGV.Request.ASSEMBLY_BACK: "assembly_back",
        MoveAGV.Request.WAREHOUSE: "warehouse",
    }

    _quad_offsets = {
        1: (0.15, 0.15),  # Quadrant 1
        2: (0.15, -0.15),  # Quadrant 2
        3: (-0.15, 0.15),  # Quadrant 3
        4: (-0.15, -0.15),  # Quadrant 4
    }

    def __init__(self):
        super().__init__("robot_controller_py")

        # ----------------------------------------------------------------------
        # Configuration Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter("motion_planning.velocity_scale", 1.0)
        self.declare_parameter("motion_planning.acceleration_scale", 0.8)
        self.declare_parameter("motion_planning.planning_time", 1.0)
        self.declare_parameter("operation_mode", "pick_place_tray")

        # ----------------------------------------------------------------------
        # Internal State Variables
        # ----------------------------------------------------------------------
        self._objects_added_to_planning_scene = False
        self._competition_start_requested = False
        self._pending_gripper_state = None
        self._gripper_state_future = None
        self._camera_image = None
        self._right_bins_camera_image = None
        self._left_bins_camera_image = None
        self._part_already_picked_up = False
        self._competition_state = None
        self._mesh_file_path = get_package_share_directory("moveit_demo") + "/meshes/"
        self._left_bins_parts = []
        self._right_bins_parts = []
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._orders = []
        self._current_order = None
        self._current_operation_mode = self.get_parameter("operation_mode").value
        self._operation_started = False
        self._competition_started = False
        self._agv_locations = {1: None, 2: None, 3: None, 4: None}
        self._kts1_trays = []
        self._kts2_trays = []
        self._conveyor_parts_expected = []
        self._conveyor_parts = []
        self._conveyor_part_detected = []
        self._floor_robot_gripper_state = None
        self._floor_robot_attached_part = None
        self._order_planning_scene_objects = []
        self._kts1_camera_received_data = False
        self._kts2_camera_received_data = False
        self._breakbeam_pose = None
        self._conveyor_speed = 0.2  # m/s

        # Constants for handling parts
        self._pick_offset = 0.003  # Offset for picking up parts
        self._drop_height = 0.01  # Height above tray for part release
        self._kit_tray_thickness = 0.01  # Thickness of the kit tray

        # ----------------------------------------------------------------------
        # Data Structures
        # ----------------------------------------------------------------------
        self._world_collision_objects = []

        self._floor_joint_positions_arrs = {
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
            self._floor_joint_positions_arrs[f"agv{i}"] = [
                self._rail_positions[f"agv{i}"],
                0.0,
                -1.57,
                1.57,
                -1.57,
                -1.57,
                0.0,
            ]
        self._floor_position_dict = {
            key: self._create_floor_joint_position_state(
                self._floor_joint_positions_arrs[key]
            )
            for key in self._floor_joint_positions_arrs.keys()
        }

        # ----------------------------------------------------------------------
        # ROS 2 Communication
        # ----------------------------------------------------------------------

        # ROS2 callback groups
        self.ariac_cb_group = MutuallyExclusiveCallbackGroup()
        self.moveit_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = ReentrantCallbackGroup()
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Service clients
        self._start_competition_client = self.create_client(
            Trigger, "/ariac/start_competition"
        )
        self._end_competition_client = self.create_client(
            Trigger, "/ariac/end_competition"
        )
        self._floor_gripper_enable = self.create_client(
            VacuumGripperControl, "/ariac/floor_robot_enable_gripper"
        )
        self._change_gripper_client = self.create_client(
            ChangeGripper,
            "/ariac/floor_robot_change_gripper",
            callback_group=self._reentrant_cb_group,
        )
        self._get_cartesian_path_client = self.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            callback_group=self.service_cb_group,
        )

        self._quality_checker = self.create_client(
            PerformQualityCheck, "/ariac/perform_quality_check"
        )

        # AGV tray lock clients
        self._lock_agv_tray_clients = {}
        self._unlock_agv_tray_clients = {}
        for i in range(1, 5):
            self._lock_agv_tray_clients[i] = self.create_client(
                Trigger, f"/ariac/agv{i}_lock_tray"
            )
            self._unlock_agv_tray_clients[i] = self.create_client(
                Trigger, f"/ariac/agv{i}_unlock_tray"
            )

        # Subscribers
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            "/ariac/competition_state",
            self._competition_state_cb,
            10,
            callback_group=self._reentrant_cb_group,
        )
        self._floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            "/ariac/floor_robot_gripper_state",
            self._floor_robot_gripper_state_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )
        self._left_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/left_bins_camera/image",
            self._left_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )
        self._right_bins_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/right_bins_camera/image",
            self._right_bins_camera_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )
        self._kts1_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/kts1_camera/image",
            self._kts1_camera_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )
        self._kts2_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            "/ariac/sensors/kts2_camera/image",
            self._kts2_camera_cb,
            qos_profile_sensor_data,
            callback_group=self._reentrant_cb_group,
        )

        self._orders_sub = self.create_subscription(
            OrderMsg,
            "/ariac/orders",
            self._orders_cb,
            10,
            callback_group=self._reentrant_cb_group,
        )

        # Parameter event subscriber
        # self._parameter_event_sub = self.create_subscription(
        #     ParameterEvent,
        #     "/parameter_events",
        #     self._parameter_event_cb,
        #     10,
        #     callback_group=self.ariac_cb_group,
        # )

        # ----------------------------------------------------------------------
        # MoveIt 2 Setup
        # ----------------------------------------------------------------------
        self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
        self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())
        self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
        self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()

        # ----------------------------------------------------------------------
        # Timers
        # ----------------------------------------------------------------------
        self._check_competition_ready_timer = self.create_timer(
            0.5, self._check_competition_ready
        )
        self._control_timer = self.create_timer(2.0, self._control_cb)

        # ----------------------------------------------------------------------
        # Part Detection
        # ----------------------------------------------------------------------
        # self.load_part_templates()

        FancyLog.info(
            self.get_logger(),
            f"Robot controller initialized with operation mode: {self._current_operation_mode}",
        )

    # def _parameter_event_cb(self, msg: ParameterEvent):
    #     """Handle parameter change events"""
    #     if msg.node == self.get_fully_qualified_name():
    #         for changed_param in msg.changed_parameters:
    #             if changed_param.name == "operation_mode":
    #                 new_mode = changed_param.value.string_value
    #                 if new_mode != self._current_operation_mode:
    #                     self.get_logger().info(
    #                         f"Operation mode changed from {self._current_operation_mode} to {new_mode}"
    #                     )
    #                     self._current_operation_mode = new_mode

    #                     # Start the operation if competition is already running
    #                     if self._competition_state == CompetitionStateMsg.STARTED:
    #                         self._start_operation()

    def _orders_cb(self, msg: OrderMsg):
        """Callback for receiving orders"""
        self._orders.append(msg)
        self.get_logger().info(f"Received order: {msg.id}, type: {msg.type}")

    def _kts1_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        """Callback for KTS1 camera images

        This function processes incoming camera messages from the KTS1 (kitting tray station 1)
        camera, storing tray poses and the camera's pose for later use in pick and place operations.

        Args:
            msg (AdvancedLogicalCameraImageMsg): Camera data containing tray information
        """
        # Store incoming tray information
        self._kts1_trays = msg.tray_poses
        self._kts1_camera_pose = msg.sensor_pose

        # Log first data received
        if (
            not hasattr(self, "_kts1_camera_received_data")
            or not self._kts1_camera_received_data
        ):
            self.get_logger().debug("Received data from KTS1 camera")
            self._kts1_camera_received_data = True

        # Debug log with tray information
        if self._kts1_trays:
            for tray in self._kts1_trays:
                self.get_logger().debug(
                    f"KTS1 detected tray ID: {tray.id} at pose: "
                    f"({tray.pose.position.x:.2f}, {tray.pose.position.y:.2f}, {tray.pose.position.z:.2f})"
                )

    def _kts2_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
        """Callback for KTS2 camera images

        This function processes incoming camera messages from the KTS2 (kitting tray station 2)
        camera, storing tray poses and the camera's pose for later use in pick and place operations.

        Args:
            msg (AdvancedLogicalCameraImageMsg): Camera data containing tray information
        """
        # Store incoming tray information
        self._kts2_trays = msg.tray_poses
        self._kts2_camera_pose = msg.sensor_pose

        # Log first data received
        if (
            not hasattr(self, "_kts2_camera_received_data")
            or not self._kts2_camera_received_data
        ):
            self.get_logger().debug("Received data from KTS2 camera")
            self._kts2_camera_received_data = True

        # Debug log with tray information
        if self._kts2_trays:
            for tray in self._kts2_trays:
                self.get_logger().debug(
                    f"KTS2 detected tray ID: {tray.id} at pose: "
                    f"({tray.pose.position.x:.2f}, {tray.pose.position.y:.2f}, {tray.pose.position.z:.2f})"
                )

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
                self._competition_started = True
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
            success = self._add_models_to_planning_scene()
            if success:
                FancyLog.pscene(
                    self.get_logger(),
                    "Successfully added all objects to planning scene",
                )
                self._objects_added_to_planning_scene = True
            else:
                FancyLog.warn(
                    self.get_logger(),
                    "Failed to add all objects to planning scene, will retry",
                )
                return  # Exit early to retry on next timer callback

        if not self._operation_started:
            # Wait for camera data to be available
            if not self._right_bins_parts:
                self.get_logger().info("Waiting for right bins camera data...")
                return
            if not self._left_bins_parts:
                self.get_logger().info("Waiting for left bins camera data...")
                return

            # Only proceed if competition is in the correct state
            if (
                self._competition_state
                in [
                    CompetitionStateMsg.STARTED,
                    CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE,
                ]
                and len(self._orders) > 0
            ):
                self._start_operation()
                self._operation_started = True

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
            # self.get_logger().info(
            #     f"Planning scene has these tracked objects: {existing_ids}"
            # )

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

    def _add_models_to_planning_scene(self):
        """
        Adds collision models to the MoveIt planning scene

        Populates the planning scene with collision objects for:
        - Bins (bins 1-8)
        - Assembly stations (AS1-AS4)
        - Assembly station briefcases/inserts
        - Conveyor belt
        - Kit tray tables (KTS1 and KTS2)

        These collision objects enable path planning with collision avoidance.
        """
        FancyLog.pscene(
            self.get_logger(), "Initializing planning scene with collision objects"
        )
        
        # Start with success as True and maintain it only if all operations succeed
        success = True
        
        # Add bins
        bin_positions = {
            "bin1": (-1.9, 3.375),
            "bin2": (-1.9, 2.625),
            "bin3": (-2.65, 2.625),
            "bin4": (-2.65, 3.375),
            "bin5": (-1.9, -3.375),
            "bin6": (-1.9, -2.625),
            "bin7": (-2.65, -2.625),
            "bin8": (-2.65, -3.375),
        }

        bin_pose = Pose()
        for bin_name, position in bin_positions.items():
            bin_pose.position.x = position[0]
            bin_pose.position.y = position[1]
            bin_pose.position.z = 0.0
            bin_pose.orientation = quaternion_from_euler(0.0, 0.0, 3.14159)

            # Aggregate success status
            success = success and self._add_model_to_planning_scene(bin_name, "bin.stl", bin_pose)

        # Add assembly stations
        assembly_station_positions = {
            "as1": (-7.3, 3.0),
            "as2": (-12.3, 3.0),
            "as3": (-7.3, -3.0),
            "as4": (-12.3, -3.0),
        }

        assembly_station_pose = Pose()
        for station_name, position in assembly_station_positions.items():
            assembly_station_pose.position.x = position[0]
            assembly_station_pose.position.y = position[1]
            assembly_station_pose.position.z = 0.0
            assembly_station_pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

            # Aggregate success status
            success = success and self._add_model_to_planning_scene(
                station_name, "assembly_station.stl", assembly_station_pose
            )

        # Add assembly briefcases
        assembly_inserts = {
            "as1_insert": "as1_insert_frame",
            "as2_insert": "as2_insert_frame",
            "as3_insert": "as3_insert_frame",
            "as4_insert": "as4_insert_frame",
        }

        for insert_name, frame_id in assembly_inserts.items():
            try:
                insert_pose = self._frame_world_pose(frame_id)
                insert_success = self._add_model_to_planning_scene(
                    insert_name, "assembly_insert.stl", insert_pose
                )
                # Aggregate success status
                success = success and insert_success
            except Exception as e:
                FancyLog.warn(self.get_logger(),
                    f"Failed to add assembly insert {insert_name}: {e}"
                )
                # Mark failure but continue with other objects
                success = False

        # Add conveyor
        conveyor_pose = Pose()
        conveyor_pose.position.x = -0.6
        conveyor_pose.position.y = 0.0
        conveyor_pose.position.z = 0.0
        conveyor_pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

        # Aggregate success status
        success = success and self._add_model_to_planning_scene("conveyor", "conveyor.stl", conveyor_pose)

        # Add kit tray tables
        kts1_table_pose = Pose()
        kts1_table_pose.position.x = -1.3
        kts1_table_pose.position.y = -5.84
        kts1_table_pose.position.z = 0.0
        kts1_table_pose.orientation = quaternion_from_euler(0.0, 0.0, 3.14159)

        # Aggregate success status
        success = success and self._add_model_to_planning_scene(
            "kts1_table", "kit_tray_table.stl", kts1_table_pose
        )

        kts2_table_pose = Pose()
        kts2_table_pose.position.x = -1.3
        kts2_table_pose.position.y = 5.84
        kts2_table_pose.position.z = 0.0
        kts2_table_pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

        # Aggregate success status
        success = success and self._add_model_to_planning_scene(
            "kts2_table", "kit_tray_table.stl", kts2_table_pose
        )

        if success:
            FancyLog.pscene(self.get_logger(), "Planning scene initialization complete")
            self._refresh_planning_scene_display()
            # time.sleep(0.5)
        else:
            FancyLog.error(self.get_logger(), "Planning scene initialization incomplete - some objects failed to load")
        
        return success

    def _start_operation(self):
        """Start the selected operation based on the current mode."""

        FancyLog.info(
            self.get_logger(), f"Starting operation: {self._current_operation_mode}"
        )

        # Add planning scene objects here, just like in the C++ code
        # self._add_models_to_planning_scene()

        if self._current_operation_mode == "pick_place_tray":
            FancyLog.info(self.get_logger(), "Executing tray pick and place operation")
            self._execute_pick_place_tray_operation()
        elif self._current_operation_mode == "pick_place_part":
            FancyLog.info(self.get_logger(), "Executing part pick and place operation")
            self._execute_pick_place_part_operation()
        else:
            FancyLog.error(
                self.get_logger(),
                f"Unknown operation mode: {self._current_operation_mode}",
            )

    def _execute_pick_place_tray_operation(self):
        """Execute tray pick and place operation."""

        # Get the first order
        if self._orders:
            self._current_order = self._orders[0]

        # Only proceed if it's a kitting task
        if self._current_order.type != OrderMsg.KITTING:
            self.get_logger().error("Order is not a kitting task")
            return False

        # Move floor robot to safe starting position
        # self._move_floor_robot_to_joint_position("home")

        # Ensure AGV is at kitting station
        agv_num = self._current_order.kitting_task.agv_number

        # Pick and place the tray only
        success = self._floor_robot_pick_and_place_tray(
            self._current_order.kitting_task.tray_id, agv_num
        )

        if success:
            self.get_logger().info("Successfully picked and placed tray")
        else:
            self.get_logger().error("Failed to pick and place tray")

        # Return to home position
        self._move_floor_robot_to_joint_position("home")

        # End competition when done
        self._end_competition()

        return success

    def _execute_pick_place_part_operation(self):
        """Execute part pick and place operation."""

        # Get the first order
        if self._orders:
            self._current_order = self._orders[0]

        # Only proceed if it's a kitting task
        if self._current_order.type != OrderMsg.KITTING:
            self.get_logger().error("Order is not a kitting task")
            return False

        # Move floor robot to safe starting position
        # self._move_floor_robot_to_joint_position("home")

        # Ensure AGV is at kitting station and has a tray
        agv_num = self._current_order.kitting_task.agv_number

        # Select a random part to pick up
        part_to_pick = PartMsg()
        found_part = False

        # Check left bins first
        if self._left_bins_parts:
            # Pick a random part from left bins
            random_index = random.randint(0, len(self._left_bins_parts) - 1)
            part_to_pick = self._left_bins_parts[random_index].part
            found_part = True

        # If no part in left bins, check right bins
        if not found_part and self._right_bins_parts:
            # Pick a random part from right bins
            random_index = random.randint(0, len(self._right_bins_parts) - 1)
            part_to_pick = self._right_bins_parts[random_index].part
            found_part = True

        if not found_part:
            self.get_logger().error("No parts found in bins")
            return False

        self.get_logger().info(
            f"Selected a {self._part_colors[part_to_pick.color]} "
            f"{self._part_types[part_to_pick.type]} part to pick"
        )

        # Pick the part
        if not self._floor_robot_pick_bin_part(part_to_pick):
            self.get_logger().error("Failed to pick part from bins")
            return False

        # Place on a random quadrant
        quadrant = random.randint(1, 4)
        self.get_logger().info(f"Placing part in quadrant {quadrant}")

        success = self._floor_robot_place_part_on_kit_tray(agv_num, quadrant)

        if success:
            self.get_logger().info("Successfully placed part on tray")
        else:
            self.get_logger().error("Failed to place part on tray")

        # Return to home position
        self._move_floor_robot_to_joint_position("home")

        # End competition when done
        self._end_competition()

        return success

    def _floor_robot_pick_and_place_tray(self, tray_id, agv_num):
        """Pick a tray and place it on the specified AGV.

        Args:
            tray_id (int): ID of the tray to pick
            agv_num (int): AGV number to place the tray on

        Returns:
            bool: True if successful, False otherwise
        """
        FancyLog.info(
            self.get_logger(),
            f"Attempting to pick up kit tray {tray_id} and place on AGV {agv_num}",
        )

        # Check if kit tray is on one of the two tables
        tray_pose = None
        station = None
        found_tray = False

        # Check table 1
        for tray in self._kts1_trays:
            if tray.id == tray_id:
                station = "kts1"
                tray_pose = multiply_pose(self._kts1_camera_pose, tray.pose)
                found_tray = True
                break

        # Check table 2 if not found on table 1
        if not found_tray:
            for tray in self._kts2_trays:
                if tray.id == tray_id:
                    station = "kts2"
                    tray_pose = multiply_pose(self._kts2_camera_pose, tray.pose)
                    found_tray = True
                    break

        if not found_tray:
            FancyLog.error(
                self.get_logger(),
                f"Attempting to pick up kit tray {tray_id} and place on AGV {agv_num}",
            )

            return False

        tray_rotation = rpy_from_quaternion(tray_pose.orientation)[2]

        self.get_logger().info(
            f"Found kit tray {tray_id} on kitting tray station {station} moving to pick location"
        )

        # Move floor robot to the corresponding kit tray table
        joint_position_key = f"floor_{station}_js_"
        self._move_floor_robot_to_joint_position(joint_position_key)

        # Change gripper to tray gripper if needed
        if self._floor_robot_gripper_state.type != "tray_gripper":
            gripper_changed = self._floor_robot_change_gripper(station, "trays")
            if not gripper_changed:
                FancyLog.error(self.get_logger(), "Failed to change to tray gripper")
                return False

        # Move to pick up tray
        waypoints = []
        gripper_orientation = quaternion_from_euler(0.0, pi, tray_rotation)

        # First move above the tray
        self._move_floor_robot_to_pose(
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + 0.5,
                gripper_orientation,
            )
        )

        # Move to grasp position
        waypoints = [
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + self._pick_offset,
                gripper_orientation,
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.2, 0.2, False)

        # Enable gripper and wait for attachment
        self._set_floor_robot_gripper_state(True)

        try:
            self._floor_robot_wait_for_attach(30.0, gripper_orientation)
        except Error as e:
            FancyLog.error(self.get_logger(), f"Failed to attach to tray: {str(e)}")

        # Lift tray
        waypoints.clear()
        waypoints.append(
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + 0.3,
                gripper_orientation,
            )
        )

        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, True)

        # Move to AGV
        agv_tray_pose = self._frame_world_pose(f"agv{agv_num}_tray")
        agv_yaw = rpy_from_quaternion(agv_tray_pose.orientation)[2]
        agv_rotation = quaternion_from_euler(0.0, pi, agv_yaw)

        # Move above AGV
        self._move_floor_robot_to_pose(
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.5,
                agv_rotation,
            )
        )

        # Lower the arm
        waypoints = [
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.01,
                agv_rotation,
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)

        # Release the tray
        self._set_floor_robot_gripper_state(False)

        # Lock tray to AGV
        self._lock_agv_tray(agv_num)

        # Move up
        waypoints.clear()
        waypoints.append(
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.3,
                quaternion_from_euler(0.0, pi, 0),
            )
        )

        self._move_floor_robot_cartesian(waypoints, 0.2, 0.2, False)

        self.get_logger().info(f"Successfully placed tray on AGV {agv_num}")
        return True

    def _lock_agv_tray(self, agv_num):
        """Lock tray to AGV."""
        self.get_logger().info(f"Locking tray to AGV {agv_num}")

        if agv_num not in self._lock_agv_tray_clients:
            self.get_logger().error(f"No lock tray client for AGV {agv_num}")
            return False

        client = self._lock_agv_tray_clients[agv_num]
        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(
            lambda future: self._lock_agv_tray_callback(future, agv_num)
        )

        # Wait a moment for the lock to take effect
        time.sleep(0.5)
        return True

    def _lock_agv_tray_callback(self, future, agv_num):
        """Callback for lock tray service."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Successfully locked tray to AGV {agv_num}")
            else:
                self.get_logger().warn(f"Failed to lock tray to AGV {agv_num}")
        except Exception as e:
            self.get_logger().error(f"Error calling lock tray service: {str(e)}")

    def _unlock_agv_tray(self, agv_num):
        """Unlock tray from AGV."""
        self.get_logger().info(f"Unlocking tray from AGV {agv_num}")

        if agv_num not in self._unlock_agv_tray_clients:
            self.get_logger().error(f"No unlock tray client for AGV {agv_num}")
            return False

        client = self._unlock_agv_tray_clients[agv_num]
        request = Trigger.Request()

        future = client.call_async(request)
        future.add_done_callback(
            lambda future: self._unlock_agv_tray_callback(future, agv_num)
        )

        return True

    def _unlock_agv_tray_callback(self, future, agv_num):
        """Callback for unlock tray service."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Successfully unlocked tray from AGV {agv_num}")
            else:
                self.get_logger().warn(f"Failed to unlock tray from AGV {agv_num}")
        except Exception as e:
            self.get_logger().error(f"Error calling unlock tray service: {str(e)}")

    def _floor_robot_place_part_on_kit_tray(self, agv_num, quadrant):
        """Place a part on a kit tray on the specified AGV in the given quadrant."""
        if (
            not self._floor_robot_gripper_state
            or not self._floor_robot_gripper_state.attached
        ):
            self.get_logger().error("No part attached")
            return False

        self.get_logger().info(f"Placing part on AGV {agv_num} in quadrant {quadrant}")

        # Validate inputs
        if agv_num < 1 or agv_num > 4:
            self.get_logger().error(f"Invalid AGV number: {agv_num}")
            return False

        if quadrant < 1 or quadrant > 4:
            self.get_logger().error(f"Invalid quadrant number: {quadrant}")
            return False

        # Move to AGV using planning scene monitor
        with self._planning_scene_monitor.read_write() as scene:
            # Set the start state
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            # Create a new state for the goal
            goal_state = copy(scene.current_state)

            # Set joint positions manually
            goal_state.joint_positions = {
                "linear_actuator_joint": self._rail_positions[f"agv{agv_num}"],
                "floor_shoulder_pan_joint": 0.0,
                # Set other joints to reasonable values
                "floor_shoulder_lift_joint": -1.0,
                "floor_elbow_joint": 1.57,
                "floor_wrist_1_joint": -1.57,
                "floor_wrist_2_joint": -1.57,
                "floor_wrist_3_joint": 0.0,
            }

            # Create constraint
            joint_constraint = construct_joint_constraint(
                robot_state=goal_state,
                joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group(
                    "floor_robot"
                ),
            )

            # Set goal
            self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])

        # Plan and execute
        success = self._plan_and_execute(
            self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"
        )

        if not success:
            self.get_logger().error("Failed to move to AGV")
            return False

        # Continue with placing the part...
        try:
            # Get the AGV tray pose
            agv_tray_pose = self._frame_world_pose(f"agv{agv_num}_tray")

            # Calculate drop position using quadrant offset
            offset_x, offset_y = self._quad_offsets[quadrant]

            # Create cartesian path to place the part
            waypoints = []
            waypoints.append(
                build_pose(
                    agv_tray_pose.position.x + offset_x,
                    agv_tray_pose.position.y + offset_y,
                    agv_tray_pose.position.z + 0.2,  # First move above
                    quaternion_from_euler(0.0, pi, 0.0),
                )
            )

            if not self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, True):
                self.get_logger().error("Failed to move above drop position")
                return False

            # Move down to place the part
            waypoints = []
            waypoints.append(
                build_pose(
                    agv_tray_pose.position.x + offset_x,
                    agv_tray_pose.position.y + offset_y,
                    agv_tray_pose.position.z + 0.05,  # Final placement position
                    quaternion_from_euler(0.0, pi, 0.0),
                )
            )

            if not self._move_floor_robot_cartesian(waypoints, 0.2, 0.2, True):
                self.get_logger().error("Failed to move to place position")
                return False

            # Release part
            self._set_floor_robot_gripper_state(False)
            time.sleep(0.5)  # Wait for release

            # Move up
            waypoints = []
            waypoints.append(
                build_pose(
                    agv_tray_pose.position.x + offset_x,
                    agv_tray_pose.position.y + offset_y,
                    agv_tray_pose.position.z + 0.2,
                    quaternion_from_euler(0.0, pi, 0.0),
                )
            )

            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, True)

            self.get_logger().info(
                f"Successfully placed part on AGV {agv_num} in quadrant {quadrant}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Error placing part: {str(e)}")
            return False


    def _create_mesh_collision_object(self, name, mesh_path, pose, frame_id="world"):
        """Create a collision object from a mesh file with better error handling."""
        try:
            # Create the collision object
            co = CollisionObject()
            co.id = name
            co.header.frame_id = frame_id
            co.header.stamp = self.get_clock().now().to_msg()

            # Load the mesh
            with pyassimp.load(mesh_path) as scene:
                if not scene.meshes:
                    self.get_logger().error(f"No meshes found in {mesh_path}")
                    return None

                mesh = Mesh()
                # Add triangles
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

                # Add vertices
                for vertex in scene.meshes[0].vertices:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    mesh.vertices.append(point)

            # Add the mesh to the collision object
            co.meshes.append(mesh)
            co.mesh_poses.append(pose)
            co.operation = CollisionObject.ADD

            return co

        except Exception as e:
            self.get_logger().error(f"Error creating collision object {name}: {str(e)}")
            return None

    def _add_model_to_planning_scene(
        self, name: str, mesh_file: str, model_pose: Pose, frame_id="world"
    ) -> bool:
        """
        Add a mesh model to the planning scene using the PlanningSceneMonitor.

        Args:
            name: Name of the collision object
            mesh_file: Path to the mesh file
            model_pose: Pose of the model
            frame_id: Frame ID for the collision object

        Returns:
            bool: True if successful, False otherwise
        """
        FancyLog.pscene(self.get_logger(),f"Adding model {name} to planning scene")

        try:
            # Get the full path to the mesh file
            model_path = self._mesh_file_path + mesh_file

            # Check if the mesh file exists
            if not path.exists(model_path):
                FancyLog.error(self.get_logger(), f"Mesh file not found: {model_path}")
                return False

            # Create collision object
            collision_object = self._make_mesh(name, model_pose, model_path, frame_id)

            if collision_object is None:
                FancyLog.error(
                    self.get_logger(), f"Failed to create collision object for {name}"
                )
                return False

            # Add to planning scene using the monitor
            with self._planning_scene_monitor.read_write() as scene:
                # Apply the collision object
                scene.apply_collision_object(collision_object)

                # Update the scene
                scene.current_state.update()

            # Add to our tracking list for later reference
            self._world_collision_objects.append(collision_object)

            FancyLog.info(
                self.get_logger(), f"Successfully added {name} to planning scene"
            )
            return True

        except Exception as e:
            FancyLog.error(
                self.get_logger(),
                f"Error adding model {name} to planning scene: {str(e)}",
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

        # # If competition just started, start the operation
        # if not self._operation_started and (
        #     self._competition_state == CompetitionStateMsg.STARTED
        #     or self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE
        # ):
        #     self._start_operation()
        #     self._operation_started = True

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

    def _set_floor_robot_gripper_state(self, state):
        """Optimized gripper state control with faster response handling"""
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().debug(f"Gripper is already {self._gripper_states[state]}")
            return None

        request = VacuumGripperControl.Request()
        request.enable = state

        # Store state for use in callback
        self._pending_gripper_state = state

        # Log at debug level instead of info
        self.get_logger().debug(
            f"Changing gripper state to {self._gripper_states[state]}"
        )

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
                controllers=["floor_robot_controller", "linear_rail_controller"],
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

        # First try waiting a short time for attachment without moving
        time.sleep(0.1)
        if self._floor_robot_gripper_state.attached:
            self.get_logger().info("Part attached on first attempt")
            return True

        # while not self._floor_robot_gripper_state.attached and retry_count < max_retries:
        while not self._floor_robot_gripper_state.attached:
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

    def _apply_planning_scene(self, scene):
        """Apply a planning scene with better timeout handling."""
        try:
            # Create a client for the service
            apply_planning_scene_client = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene"
            )

            # Wait for service to be available with reduced timeout
            if not apply_planning_scene_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("'/apply_planning_scene' service not available")
                return False

            # Create and send the request
            request = ApplyPlanningScene.Request()
            request.scene = scene

            # Send the request
            future = apply_planning_scene_client.call_async(request)

            # Wait for the response with shorter timeout
            timeout_sec = 1.0  # Reduced timeout
            start_time = time.time()

            while not future.done():
                time.sleep(0.01)  # Shorter sleep interval

                if time.time() - start_time > timeout_sec:
                    self.get_logger().warn(
                        "Timeout waiting for planning scene service, trying direct publish instead"
                    )
                    # Try direct publishing as a fallback
                    self._direct_publish_planning_scene(scene)
                    return False  # Assume success with direct publishing

            # Process the result
            result = future.result()
            if result.success:
                self.get_logger().info("Successfully applied planning scene")
                return True
            else:
                self.get_logger().warn(
                    "Failed to apply planning scene via service"
                )
                # Try direct publishing as a fallback
                # self._direct_publish_planning_scene(scene)
                return True

        except Exception as e:
            self.get_logger().error(f"Error applying planning scene: {str(e)}")
            # Try direct publishing as a fallback
            try:
                self._direct_publish_planning_scene(scene)
                return True
            except Exception as e2:
                self.get_logger().error(f"Direct publishing also failed: {str(e2)}")
                return False

    def _direct_publish_planning_scene(self, scene):
        """Publish planning scene directly to the planning scene topic."""
        if not hasattr(self, "_planning_scene_publisher"):
            self._planning_scene_publisher = self.create_publisher(
                PlanningScene, "/planning_scene", 10
            )
            # Short delay to allow publisher to initialize
            time.sleep(0.1)

        # Set is_diff flag for proper scene update
        scene.is_diff = True

        # Publish the scene
        self._planning_scene_publisher.publish(scene)

        # Give some time for the planning scene to be processed
        time.sleep(0.1)

        self.get_logger().info(
            f"Directly published planning scene with {len(scene.world.collision_objects)} objects"
        )

    def _refresh_planning_scene_display(self):
        """Force a refresh of the planning scene display in RViz."""
        # Create an empty diff planning scene just to trigger a display update
        refresh_scene = PlanningScene()
        refresh_scene.is_diff = True
        self._apply_planning_scene(refresh_scene)

    def _remove_model_from_floor_gripper(self):
        self.get_logger().info("Removing attached part from floor gripper")
        temp_scene = copy(self.planning_scene_msg)
        with self._planning_scene_monitor.read_write() as scene:
            temp_scene.world.collision_objects = self._world_collision_objects
            temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
            temp_scene.robot_state.attached_collision_objects.clear()
            self._apply_planning_scene(temp_scene)
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

    def _floor_robot_move_to_target(self):
        """Plan and execute movement to the current target

        Returns:
            bool: True if successful, False otherwise
        """
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"Planning attempt {attempt} of {max_attempts}")

            # Plan the motion with default parameters
            plan_result = self._floor_robot.plan()

            # If plan is found, execute it
            if plan_result:
                self.get_logger().info("Plan found, executing...")

                # For MoveIt2 with ROS2 Iron:
                # The PlanningComponent doesn't have an execute method
                # We need to use the parent MoveItPy object to execute
                execution_success = self._ariac_robots.execute(
                    plan_result.trajectory,
                    controllers=["floor_robot_controller", "linear_rail_controller"],
                )

                if execution_success:
                    self.get_logger().info("Plan execution succeeded")
                    return True
                self.get_logger().warn(
                    f"Plan execution failed on attempt {attempt} of {max_attempts}"
                )
            else:
                self.get_logger().warn(
                    f"Unable to generate plan on attempt {attempt} of {max_attempts}"
                )

            # Small delay before retry
            time.sleep(0.5)

        self.get_logger().error(
            f"Failed to move to target after {max_attempts} attempts"
        )
        return False

    def _move_floor_robot_to_joint_position(self, position_name: str):
        """Move the floor robot to a predefined joint position."""
        self.get_logger().info(f"Moving to position: {position_name}")

        try:
            with self._planning_scene_monitor.read_write() as scene:
                # Set the start state
                self._floor_robot.set_start_state(robot_state=scene.current_state)

                # Handle different position types
                if position_name == "home":
                    # For home, we use predefined values
                    home_values = {
                        "linear_actuator_joint": 0.0,
                        "floor_shoulder_pan_joint": 0.0,
                        "floor_shoulder_lift_joint": -1.57,
                        "floor_elbow_joint": 1.57,
                        "floor_wrist_1_joint": -1.57,
                        "floor_wrist_2_joint": -1.57,
                        "floor_wrist_3_joint": 0.0,
                    }

                    # Create a new state for the goal
                    goal_state = copy(scene.current_state)
                    goal_state.joint_positions = home_values

                elif position_name in self._floor_position_dict:
                    # Create a new state for the goal
                    goal_state = copy(scene.current_state)
                    goal_state.joint_positions = self._floor_position_dict[
                        position_name
                    ]

                else:
                    self.get_logger().error(f"Position '{position_name}' not found")
                    return False

                # Create constraint
                joint_constraint = construct_joint_constraint(
                    robot_state=goal_state,
                    joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group(
                        "floor_robot"
                    ),
                )

                # Set goal
                self._floor_robot.set_goal_state(
                    motion_plan_constraints=[joint_constraint]
                )

            # Plan and execute
            success = self._plan_and_execute(
                self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"
            )

            if success:
                self.get_logger().info(f"Successfully moved to {position_name}")
                return True
            else:
                self.get_logger().error(f"Failed to move to {position_name}")
                return False

        except Exception as e:
            self.get_logger().error(
                f"Error moving to position '{position_name}': {str(e)}"
            )
            return False

    def _floor_robot_pick_bin_part(self, part_to_pick: PartMsg):
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
                if (
                    part.part.type == part_to_pick.type
                    and part.part.color == part_to_pick.color
                ):
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
                part_pose.position.z
                + RobotController._part_heights[part_to_pick.type]
                + 0.005,
                gripper_orientation,
            )
        ]
        # Use faster movement for approach
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

        # Enable gripper with less waiting
        self._set_floor_robot_gripper_state(True)

        # More efficient attachment process with shorter timeout
        try:
            self._floor_robot_wait_for_attach(
                10.0, gripper_orientation
            )  # Reduced timeout
        except Error as e:
            self.get_logger().error(f"Attachment failed: {str(e)}")

            # Quick recovery
            waypoints = [
                build_pose(
                    part_pose.position.x,
                    part_pose.position.y,
                    part_pose.position.z + 0.2,
                    gripper_orientation,
                )
            ]
            self._move_floor_robot_cartesian(
                waypoints, 0.5, 0.5, False
            )  # Faster retreat
            self._set_floor_robot_gripper_state(False)
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
        self._move_floor_robot_cartesian(waypoints, 0.2, 0.2, False)

        # Return to bin position
        self._move_floor_robot_to_joint_position(bin_side)

        # SCENE UPDATE PHASE - Minimal planning scene updates
        # Just record the attached part internally for tracking
        self._floor_robot_attached_part = part_to_pick

        # Only update planning scene if needed
        self._attach_model_to_floor_gripper(part_to_pick, part_pose)

        return True

    def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
        """
        Attach a part to the floor gripper in the planning scene efficiently.

        This implementation is optimized for speed, with options for debugging.
        """
        # Create a part name based on its color and type
        # Create a part name based on its color and type
        part_name = (
            self._part_colors[part_to_pick.color]
            + "_"
            + self._part_types[part_to_pick.type]
        )

        # Always track the part internally for collision awareness
        self._floor_robot_attached_part = part_to_pick

        # Check if we're using simplified mode for speed
        debug_mode = False  # Set this to True only when debugging visualization

        if not debug_mode:
            # Fast path - just track the part without scene updates
            self.get_logger().info(
                f"Tracking attached part: {part_name} (simplified for speed)"
            )
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
                                face.indices[2],
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
            self._apply_planning_scene(planning_scene)

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
            success = self._apply_planning_scene(attach_scene)

            if success:
                self.get_logger().info(
                    f"Successfully attached {part_name} to floor gripper"
                )
                return True
            else:
                self.get_logger().error(
                    f"Failed to attach {part_name} to floor gripper"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Error attaching model to gripper: {str(e)}")
            return False

    def _floor_robot_change_gripper(self, station: str, gripper_type: str):
        """Change the gripper on the floor robot.

        Args:
            station (str): Station to change gripper at ("kts1" or "kts2")
            gripper_type (str): Type of gripper to change to ("trays" or "parts")

        Returns:
            bool: True if successful, False otherwise
        """
        FancyLog.info(self.get_logger(), f"Changing gripper to type: {gripper_type}")

        try:
            # Get the pose of the tool changer frame
            tc_pose = self._frame_world_pose(
                f"{station}_tool_changer_{gripper_type}_frame"
            )
        except Exception as e:
            FancyLog.error(
                self.get_logger(), f"Failed to get tool changer frame pose: {str(e)}"
            )
            return False

        # Move above the tool changer
        self._move_floor_robot_to_pose(
            build_pose(
                tc_pose.position.x,
                tc_pose.position.y,
                tc_pose.position.z + 0.4,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        )

        # Move to the tool changer
        waypoints = [
            build_pose(
                tc_pose.position.x,
                tc_pose.position.y,
                tc_pose.position.z,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.2, 0.2, False)

        # Create and send the service request
        request = ChangeGripper.Request()

        if gripper_type == "trays":
            request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER
        elif gripper_type == "parts":
            request.gripper_type = ChangeGripper.Request.PART_GRIPPER

        # Check if service is available with timeout
        if not self._change_gripper_client.wait_for_service(timeout_sec=2.0):
            FancyLog.error(self.get_logger(), "Change gripper service not available")
            return False

        future = self._change_gripper_client.call_async(request)

        # Wait for the response with timeout (5 seconds)
        timeout_sec = 5.0
        start_time = time.time()

        while not future.done():
            if time.time() - start_time > timeout_sec:
                FancyLog.error(
                    self.get_logger(), "Timeout waiting for change gripper service"
                )
                # Move away from the tool changer to avoid being stuck
                waypoints = [
                    build_pose(
                        tc_pose.position.x,
                        tc_pose.position.y,
                        tc_pose.position.z + 0.4,
                        quaternion_from_euler(0.0, pi, 0.0),
                    )
                ]
                self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
                return False

            # Sleep briefly to avoid CPU spinning
            time.sleep(0.05)

        # Check the result
        try:
            result = future.result()
            if not result.success:
                FancyLog.error(
                    self.get_logger(),
                    "Error from change gripper service: "
                    + (
                        result.message
                        if hasattr(result, "message")
                        else "No error message"
                    ),
                )
                return False
        except Exception as e:
            FancyLog.error(
                self.get_logger(),
                f"Exception getting result from change gripper service: {str(e)}",
            )
            return False

        # Move away from the tool changer
        waypoints = [
            build_pose(
                tc_pose.position.x,
                tc_pose.position.y,
                tc_pose.position.z + 0.4,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
        FancyLog.info(
            self.get_logger(), f"Successfully changed to {gripper_type} gripper"
        )
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
            # First check if the transform is available
            if not self._tf_buffer.can_transform("world", frame_id, rclpy.time.Time()):
                self.get_logger().warn(
                    f"Transform from world to {frame_id} not immediately available, waiting..."
                )

            # Wait synchronously for the transform
            timeout = rclpy.duration.Duration(seconds=2.0)
            t = self._tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time(), timeout)
  
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
