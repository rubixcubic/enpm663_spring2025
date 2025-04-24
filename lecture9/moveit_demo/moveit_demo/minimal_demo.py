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

        self._objects_added_to_planning_scene = False
        self._pending_gripper_state = None
        self._gripper_state_future = None

        # ROS2 callback groups
        self.ariac_cb_group = MutuallyExclusiveCallbackGroup()
        self.moveit_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = ReentrantCallbackGroup()

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
            callback_group=self.service_cb_group,
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
            callback_group=self.ariac_cb_group,
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

        self._world_collision_objects = []

        # Parts found in the bins
        self._left_bins_parts = []
        self._right_bins_parts = []
        self._left_bins_camera_pose = Pose()
        self._right_bins_camera_pose = Pose()

        # service clients
        self._get_cartesian_path_client = self.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            callback_group=self.service_cb_group,
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

        self.floor_robot_attached_part_ = PartMsg()

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
        self.mesh_file_path = get_package_share_directory("moveit_demo") + "/meshes/"

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

        This function is triggered periodically by a ROS 2 timer. It checks the
        current state of the competition and the availability of parts in the bins.
        If the competition has started or order announcements are done
        and a specific part (purple pump) has not yet been picked up, it initiates
        the process of picking that part using the floor robot. It logs success or
        failure of the picking operation.
        """

        # Add models to the planning scene
        if not self._objects_added_to_planning_scene:
            self._add_objects_to_planning_scene()
            self._objects_added_to_planning_scene = True
        else:

            if not self._right_bins_parts:
                return

            if self._competition_state not in [
                CompetitionStateMsg.STARTED,
                CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE,
            ]:
                return

            # Go to different locations using Joint-space programming
            # self._move_floor_robot_to_joint_position('left_bins')
            # self._move_floor_robot_to_joint_position('right_bins')
            self._move_floor_robot_to_joint_position("agv1")
            self._move_floor_robot_to_joint_position("agv2")
            self._move_floor_robot_to_joint_position("agv3")
            self._move_floor_robot_to_joint_position("agv4")

            # Pick up a part from a bin
            if not self._part_already_picked_up:
                part_to_pick = PartMsg()
                part_to_pick.color = PartMsg.PURPLE
                part_to_pick.type = PartMsg.PUMP

                success = self.floor_robot_pick_bin_part(part_to_pick)
                if success:
                    self._part_already_picked_up = True
                    self.get_logger().info("Successfully picked up part")
                else:
                    self.get_logger().warn("Failed to pick up part, will retry later")

            # We are done, stop the competitionn
            self._end_competition()

    def _add_objects_to_planning_scene(self):
        package_share_directory = get_package_share_directory("moveit_demo")
        with open(
            package_share_directory + "/config/collision_object_info.yaml", "r"
        ) as object_file:
            objects_dict = yaml.safe_load(object_file)

        for key in objects_dict.keys():
            object_pose = Pose()

            object_pose.position.x = float(objects_dict[key]["position"][0])
            object_pose.position.y = float(objects_dict[key]["position"][1])
            object_pose.position.z = float(objects_dict[key]["position"][2])

            object_pose.orientation.x = float(objects_dict[key]["orientation"][0])
            object_pose.orientation.y = float(objects_dict[key]["orientation"][1])
            object_pose.orientation.z = float(objects_dict[key]["orientation"][2])
            object_pose.orientation.w = float(objects_dict[key]["orientation"][3])

            self._add_model_to_planning_scene(
                key, objects_dict[key]["file"], object_pose
            )

    def _add_model_to_planning_scene(
        self, name: str, mesh_file: str, model_pose: Pose, frame_id="world"
    ):
        self.get_logger().info(f"Adding {name} to planning scene")
        model_path = self.mesh_file_path + mesh_file
        collision_object = self._make_mesh(
            name, model_pose, model_path, frame_id=frame_id
        )
        with self._planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            self._world_collision_objects.append(collision_object)
            scene.current_state.update()

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
        """Set the gripper state of the floor robot.

        Arguments:
            state -- True to enable, False to disable

        Returns:
            Future: The future object for the service call
        """
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f"Gripper is already {self._gripper_states[state]}")
            return None

        request = VacuumGripperControl.Request()
        request.enable = state

        # Store state for use in callback
        self._pending_gripper_state = state

        # Use call_async with a callback
        self.get_logger().info(
            f"Requesting gripper state change to {self._gripper_states[state]}"
        )
        future = self._floor_gripper_enable.call_async(request)
        future.add_done_callback(self._gripper_state_callback)

        # Store and return the future so it can be waited on if needed
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
        trajectory_msg = self._call_get_cartesian_path(
            waypoints, velocity, acceleration, avoid_collision, "floor_robot"
        )
        with self._planning_scene_monitor.read_write() as scene:
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = "floor_robot"
            scene.current_state.update(True)
            self._ariac_robots_state = scene.current_state

        self._ariac_robots.execute(trajectory, controllers=[])

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

    # def _call_get_cartesian_path(self, waypoints : list,
    #                           max_velocity_scaling_factor : float,
    #                           max_acceleration_scaling_factor : float,
    #                           avoid_collision : bool,
    #                           robot : str):
    #     self.get_logger().info("Getting cartesian path")

    #     request = GetCartesianPath.Request()
    #     header = Header()
    #     header.frame_id = "world"
    #     header.stamp = self.get_clock().now().to_msg()
    #     request.header = header

    #     with self._planning_scene_monitor.read_write() as scene:
    #         request.start_state = robotStateToRobotStateMsg(scene.current_state)

    #     if robot == "floor_robot":
    #         request.group_name = "floor_robot"
    #         request.link_name = "floor_gripper"
    #     else:
    #         request.group_name = "ceiling_robot"
    #         request.link_name = "ceiling_gripper"

    #     request.waypoints = waypoints
    #     request.max_step = 0.1
    #     request.avoid_collisions = avoid_collision
    #     request.max_velocity_scaling_factor = max_velocity_scaling_factor
    #     request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

    #     # Use a timeout to prevent hanging
    #     start_time = time.time()
    #     timeout = 5.0  # 5 second timeout

    #     future = self._get_cartesian_path_client.call_async(request)

    #     # Wait with timeout
    #     while not future.done():
    #         time.sleep(0.01)  # Sleep briefly to allow other callbacks to run
    #         if time.time() - start_time > timeout:
    #             self.get_logger().error(f"Cartesian path service timeout after {timeout} seconds")
    #             return None

    #     try:
    #         result = future.result()

    #         if result.fraction < 0.9:
    #             self.get_logger().error(f"Unable to plan full cartesian trajectory (fraction: {result.fraction:.2f})")

    #         return result.solution
    #     except Exception as e:
    #         self.get_logger().error(f"Error getting cartesian path: {str(e)}")
    #         return None

    def _call_get_cartesian_path(
        self,
        waypoints: list,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        avoid_collision: bool,
        robot: str,
    ):
        self.get_logger().info("Getting cartesian path")

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

        request.waypoints = waypoints
        request.max_step = 0.1
        request.avoid_collisions = avoid_collision
        request.max_velocity_scaling_factor = max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        future = self._get_cartesian_path_client.call_async(request)

        while not future.done():
            pass

        result: GetCartesianPath.Response
        result = future.result()

        if result.fraction < 0.9:
            self.get_logger().error("Unable to plan cartesian trajectory")

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
        """Helper function to plan and execute a motion.

        Args:
            robot: The robot to move
            planning_component: The planning component to use
            logger: Logger to use
            robot_type (str): Type of robot (floor_robot or ceiling_robot)
            single_plan_parameters: Parameters for single plan
            multi_plan_parameters: Parameters for multi plan
            sleep_time (float): Time to sleep after execution

        Returns:
            bool: True if successful, False otherwise
        """
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update(True)
                self._ariac_robots_state = scene.current_state
                robot_trajectory = plan_result.trajectory
            robot.execute(
                robot_trajectory,
                controllers=["floor_robot_controller", "linear_rail_controller"]
                if robot_type == "floor_robot"
                else ["ceiling_robot_controller", "gantry_controller"],
            )
        else:
            logger.error("Planning failed")
            return False
        return True

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
        with self._planning_scene_monitor.read_write() as scene:
            current_pose = scene.current_state.get_pose("floor_gripper")
        start_time = time.time()
        while not self._floor_robot_gripper_state.attached:
            current_pose = build_pose(
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z - 0.001,
                orientation,
            )
            waypoints = [current_pose]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
            # time.sleep(0.2)
            if time.time() - start_time >= timeout:
                self.get_logger().error("Unable to pick up part")

    # def _floor_robot_wait_for_attach(self, timeout: float, orientation: Quaternion):
    #     """More robust attachment approach that continues even if movement fails."""
    #     # Make sure gripper is enabled
    #     if not self._floor_robot_gripper_state.enabled:
    #         self.get_logger().info("Enabling gripper")
    #         self.set_floor_robot_gripper_state(True)
    #         time.sleep(1.0)  # Give more time for gripper to enable

    #     # Get current pose
    #     with self._planning_scene_monitor.read_write() as scene:
    #         scene.current_state.update(True)
    #         current_pose = scene.current_state.get_pose("floor_gripper")

    #     # Initial check for attachment
    #     if self._floor_robot_gripper_state.attached:
    #         self.get_logger().info("Part already attached")
    #         return

    #     # Try several approaches to get attachment
    #     start_time = time.time()
    #     remaining_time = timeout

    #     # Log initial position
    #     self.get_logger().info(
    #         f"Initial gripper position: x={current_pose.position.x:.4f}, "
    #         f"y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}"
    #     )

    #     # First try
    #     self.get_logger().info("First attempt - 5mm down")
    #     try:
    #         # Build target pose for first movement
    #         target_pose = Pose()
    #         target_pose.position.x = current_pose.position.x
    #         target_pose.position.y = current_pose.position.y
    #         target_pose.position.z = current_pose.position.z - 0.005
    #         target_pose.orientation = orientation

    #         # Try pose-based movement (more reliable than Cartesian)
    #         with self._planning_scene_monitor.read_write() as scene:
    #             scene.current_state.update(True)
    #             self._floor_robot.set_start_state(robot_state=scene.current_state)

    #             pose_goal = PoseStamped()
    #             pose_goal.header.frame_id = "world"
    #             pose_goal.pose = target_pose
    #             self._floor_robot.set_goal_state(
    #                 pose_stamped_msg=pose_goal, pose_link="floor_gripper"
    #             )

    #         # Plan and execute immediately
    #         plan_result = self._floor_robot.plan()
    #         if plan_result:
    #             self._ariac_robots.execute(
    #                 plan_result.trajectory,
    #                 controllers=["floor_robot_controller", "linear_rail_controller"],
    #             )
    #             self.get_logger().info("Executed first movement")
    #         else:
    #             self.get_logger().warn("First movement planning failed")
    #     except Exception as e:
    #         self.get_logger().warn(f"First movement error: {str(e)}")

    #     # Wait a bit to see if attachment happened
    #     time.sleep(1.0)

    #     # Check if attached after first movement
    #     if self._floor_robot_gripper_state.attached:
    #         self.get_logger().info("Part attached after first movement")
    #         return

    #     # Calculate remaining timeout
    #     remaining_time = timeout - (time.time() - start_time)
    #     if remaining_time <= 0:
    #         self.get_logger().error("Timeout after first attempt")
    #         raise Error("Unable to pick up part within timeout")

    #     # Second try - toggle gripper and try again
    #     self.get_logger().info("Second attempt - toggling gripper and 8mm move")

    #     # Toggle gripper
    #     self.set_floor_robot_gripper_state(False)
    #     time.sleep(0.5)
    #     self.set_floor_robot_gripper_state(True)
    #     time.sleep(0.5)

    #     # Get updated pose
    #     with self._planning_scene_monitor.read_write() as scene:
    #         scene.current_state.update(True)
    #         current_pose = scene.current_state.get_pose("floor_gripper")

    #     try:
    #         # movement down
    #         target_pose = Pose()
    #         target_pose.position.x = current_pose.position.x
    #         target_pose.position.y = current_pose.position.y
    #         target_pose.position.z = current_pose.position.z - 0.003
    #         target_pose.orientation = orientation

    #         # Try using joint-space planning again
    #         with self._planning_scene_monitor.read_write() as scene:
    #             scene.current_state.update(True)
    #             self._floor_robot.set_start_state(robot_state=scene.current_state)

    #             pose_goal = PoseStamped()
    #             pose_goal.header.frame_id = "world"
    #             pose_goal.pose = target_pose
    #             self._floor_robot.set_goal_state(
    #                 pose_stamped_msg=pose_goal, pose_link="floor_gripper"
    #             )

    #         plan_result = self._floor_robot.plan()
    #         if plan_result:
    #             self._ariac_robots.execute(
    #                 plan_result.trajectory,
    #                 controllers=["floor_robot_controller", "linear_rail_controller"],
    #             )
    #             self.get_logger().info("Executed second movement")
    #         else:
    #             self.get_logger().warn("Second movement planning failed")
    #     except Exception as e:
    #         self.get_logger().warn(f"Second movement error: {str(e)}")

    #     # Final wait for attachment
    #     wait_start = time.time()
    #     remaining_time = timeout - (wait_start - start_time)

    #     self.get_logger().info(f"Final wait for up to {remaining_time:.2f} seconds")

    #     # Wait until timeout expires
    #     while time.time() - wait_start < remaining_time:
    #         if self._floor_robot_gripper_state.attached:
    #             self.get_logger().info("Part attached during final wait")
    #             return
    #         time.sleep(0.1)

    #     # If we get here, we've timed out
    #     elapsed_time = time.time() - start_time
    #     self.get_logger().error(
    #         f"Total time elapsed: {elapsed_time:.2f}s (timeout: {timeout:.2f}s)"
    #     )
    #     raise Error("Unable to pick up part within timeout")

    # def _floor_robot_wait_for_attach(self, timeout: float, orientation: Quaternion):
    #     """Simpler approach for attachment with direct joint commands."""
    #     # Make sure gripper is enabled
    #     if not self._floor_robot_gripper_state.enabled:
    #         self.get_logger().info("Enabling gripper")
    #         request = VacuumGripperControl.Request()
    #         request.enable = True
    #         self._floor_gripper_enable.call_async(request)
    #         time.sleep(1.0)  # Wait for gripper to enable

    #     # Get initial pose
    #     with self._planning_scene_monitor.read_write() as scene:
    #         scene.current_state.update(True)
    #         initial_pose = scene.current_state.get_pose("floor_gripper")

    #     # Check if already attached (sometimes it happens immediately)
    #     if self._floor_robot_gripper_state.attached:
    #         self.get_logger().info("Part already attached")
    #         return

    #     # First attempt - wait briefly without moving
    #     self.get_logger().info("Waiting for attachment without movement")
    #     start_time = time.time()
    #     initial_wait = 2.0
    #     while time.time() - start_time < initial_wait:
    #         if self._floor_robot_gripper_state.attached:
    #             self.get_logger().info("Part attached during initial wait")
    #             return
    #         time.sleep(0.1)

    #     # If not attached after initial wait, try a single large downward movement
    #     if not self._floor_robot_gripper_state.attached:
    #         z_offset = -0.01  # 1cm down - larger movement

    #         self.get_logger().info(f"Moving down 10mm for attachment")

    #         # Direct positioning using a pose goal
    #         target_pose = PoseStamped()
    #         target_pose.header.frame_id = "world"
    #         target_pose.pose.position.x = initial_pose.position.x
    #         target_pose.pose.position.y = initial_pose.position.y
    #         target_pose.pose.position.z = initial_pose.position.z + z_offset
    #         target_pose.pose.orientation = orientation

    #         try:
    #             with self._planning_scene_monitor.read_write() as scene:
    #                 scene.current_state.update(True)
    #                 self._floor_robot.set_start_state(robot_state=scene.current_state)
    #                 self._floor_robot.set_goal_state(
    #                     pose_stamped_msg=target_pose, pose_link="floor_gripper"
    #                 )

    #             plan_result = self._floor_robot.plan()
    #             if plan_result:
    #                 with self._planning_scene_monitor.read_write() as scene:
    #                     scene.current_state.update(True)
    #                     self._ariac_robots_state = scene.current_state
    #                 # Add the controllers parameter
    #                 self._ariac_robots.execute(
    #                     plan_result.trajectory,
    #                     controllers=[
    #                         "floor_robot_controller",
    #                         "linear_rail_controller",
    #                     ],
    #                 )
    #                 self.get_logger().info("Executed downward movement")
    #             else:
    #                 self.get_logger().warn("Failed to plan downward movement")
    #         except Exception as e:
    #             self.get_logger().error(f"Error in downward movement: {str(e)}")

    #     # Final wait for attachment
    #     self.get_logger().info("Final wait for attachment")
    #     final_start = time.time()
    #     while time.time() - final_start < timeout:
    #         if self._floor_robot_gripper_state.attached:
    #             self.get_logger().info("Part attached during final wait")
    #             return
    #         time.sleep(0.1)

    #     # If we get here, attachment failed
    #     self.get_logger().error("Failed to attach after timeout")
    #     raise Error("Unable to pick up part within timeout")

    # def _move_floor_robot_small_z(self, z_offset, orientation):
    #     """
    #     Move the robot down by a small amount in z-axis using joint-space planning.

    #     Args:
    #         z_offset (float): Amount to move down in meters (negative value)
    #         orientation (Quaternion): Orientation to maintain

    #     Returns:
    #         bool: True if successful, False otherwise
    #     """
    #     try:
    #         # Get current pose
    #         with self._planning_scene_monitor.read_write() as scene:
    #             scene.current_state.update(True)
    #             current_pose = scene.current_state.get_pose("floor_gripper")

    #         # Create target pose
    #         target_pose = PoseStamped()
    #         target_pose.header.frame_id = "world"
    #         target_pose.pose.position.x = current_pose.position.x
    #         target_pose.pose.position.y = current_pose.position.y
    #         target_pose.pose.position.z = current_pose.position.z + z_offset
    #         target_pose.pose.orientation = orientation

    #         # Plan directly to the pose using joint space planning
    #         with self._planning_scene_monitor.read_write() as scene:
    #             self._floor_robot.set_start_state(robot_state=scene.current_state)
    #             self._floor_robot.set_goal_state(pose_stamped_msg=target_pose, pose_link="floor_gripper")

    #         # Set shorter planning time to make it faster
    #         self._floor_robot.set_planning_time(0.5)

    #         # Plan and execute
    #         self.get_logger().info(f"Planning small move down by {-z_offset*1000:.1f}mm")
    #         plan_result = self._floor_robot.plan()

    #         if plan_result:
    #             self.get_logger().info("Executing small movement")
    #             with self._planning_scene_monitor.read_write() as scene:
    #                 scene.current_state.update(True)
    #                 self._ariac_robots_state = scene.current_state
    #                 robot_trajectory = plan_result.trajectory

    #             self._ariac_robots.execute(robot_trajectory)
    #             time.sleep(0.5)  # Wait for motion to complete
    #             return True
    #         else:
    #             self.get_logger().warn("Failed to plan small movement")
    #             return False

    #     except Exception as e:
    #         self.get_logger().error(f"Error in small z movement: {str(e)}")
    #         return False

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
                if hasattr(face, 'indices'):
                    if len(face.indices) == 3:
                        triangle.vertex_indices = [
                            face.indices[0],
                            face.indices[1],
                            face.indices[2]
                        ]
                        mesh.triangles.append(triangle)
                else:
                    if len(face) == 3:
                        triangle.vertex_indices = [
                            face[0],
                            face[1],
                            face[2]
                        ]
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
    
    def _make_attached_mesh(
        self, name, pose, filename, robot
    ) -> AttachedCollisionObject:
        """Create an attached mesh collision object.

        Args:
            name (str): Name of the mesh
            pose (Pose): Pose of the mesh
            filename (str): Path to the mesh file
            robot (str): Robot to attach the mesh to

        Returns:
            AttachedCollisionObject: The attached collision object
        """
        with pyassimp.load(filename) as scene:
            assert len(scene.meshes)

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
        else:
            o.link_name = "ceiling_gripper"
        o.object.header.frame_id = "world"
        o.object.id = name
        o.object.meshes.append(mesh)
        o.object.mesh_poses.append(pose)
        return o

    def apply_planning_scene(self, scene):
        """Apply a planning scene to the MoveIt planning scene.

        Args:
            scene (PlanningScene): Planning scene to apply

        Returns:
            bool: True if successful, False otherwise
        """
        apply_planning_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        # Create a request object.
        request = ApplyPlanningScene.Request()

        # Set the request location.
        request.scene = scene

        # Send the request.
        future = apply_planning_scene_client.call_async(request)

        # Wait for the server to respond.
        wait_count = 0
        max_wait = 30  # 3 seconds total
        while not future.done() and wait_count < max_wait:
            time.sleep(0.1)
            wait_count += 1

        if not future.done():
            self.get_logger().error("Timeout waiting for planning scene to apply")
            return False

        # Check the result of the service call.
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Successfully applied new planning scene")
                return True
            else:
                self.get_logger().warn(
                    f"Failed to apply planning scene: {result.message}"
                )
                return False
        except Exception as e:
            self.get_logger().error(f"Error applying planning scene: {str(e)}")
            return False

    def _move_floor_robot_to_joint_position(self, position_name: str):
        """
        Move the floor robot to a predefined joint position.

        Args:
            position_name (str): Name of the predefined position to move to
                                (e.g., "left_bins", "right_bins", "agv1", etc.)

        Returns:
            bool: True if successful, False otherwise
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

        # Plan and execute the motion with better error handling
        success = self._plan_and_execute(
            self._ariac_robots,
            self._floor_robot,
            self.get_logger(),
            robot_type="floor_robot",
        )
        if not success:
            self.get_logger().error(f"Failed to move to position: {position_name}")
        return success

    def floor_robot_pick_bin_part(self, part_to_pick: PartMsg):
        """
        Pick a part from a bin using Cartesian space programming.

        This function implements a complete pick operation:
        1. Locates the target part in either the left or right bins
        2. Changes the gripper if necessary
        3. Moves to the bin containing the part
        4. Approaches the part from above
        5. Moves down to grasp position
        6. Enables the gripper and waits for part attachment
        7. Moves back to a safe position with the attached part
        8. Updates the planning scene with the attached part

        Args:
            part_to_pick (PartMsg): Part to pick (containing color and type information)

        Returns:
            bool: True if the entire picking operation succeeded, False otherwise
        """
        # Log the part we're trying to pick for debugging and monitoring
        self.get_logger().info(
            f"Attempting to pick {self._part_colors[part_to_pick.color]} {self._part_types[part_to_pick.type]}"
        )

        # Initialize variables for part location
        part_pose = Pose()
        found_part = False
        bin_side = ""

        # PART DETECTION PHASE:
        # First, search for the part in the left bins
        for part in self._left_bins_parts:
            self.get_logger().debug(
                f"Checking left bin part: {part.part.type}, {part.part.color}"
            )
            # Check if this part matches what we're looking for
            if (
                part.part.type == part_to_pick.type
                and part.part.color == part_to_pick.color
            ):
                # Transform part pose from camera frame to world frame
                part_pose = multiply_pose(self._left_bins_camera_pose, part.pose)
                found_part = True
                bin_side = "left_bins"

                # Log detailed position for debugging
                roll, pitch, yaw = rpy_from_quaternion(part_pose.orientation)
                self.get_logger().debug(
                    f"Part found at position: x={part_pose.position.x:.4f}, "
                    f"y={part_pose.position.y:.4f}, z={part_pose.position.z:.4f}, "
                    f"orientation (rpy): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}"
                )
                break

        # If not found in left bins, check right bins
        if not found_part:
            for part in self._right_bins_parts:
                self.get_logger().debug(
                    f"Checking right bin part: {part.part.type}, {part.part.color}"
                )
                if (
                    part.part.type == part_to_pick.type
                    and part.part.color == part_to_pick.color
                ):
                    part_pose = multiply_pose(self._right_bins_camera_pose, part.pose)
                    found_part = True
                    bin_side = "right_bins"

                    # Log detailed position for debugging
                    roll, pitch, yaw = rpy_from_quaternion(part_pose.orientation)
                    self.get_logger().debug(
                        f"Part found at position: x={part_pose.position.x:.4f}, "
                        f"y={part_pose.position.y:.4f}, z={part_pose.position.z:.4f}, "
                        f"orientation (rpy): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}"
                    )
                    break

        # Early exit if part cannot be found
        if not found_part:
            self.get_logger().error("Unable to locate part")
            return False
        else:
            self.get_logger().info(f"Part found in {bin_side}")

        # GRIPPER PREPARATION PHASE:
        # Check if we need to change the gripper type
        if self._floor_robot_gripper_state.type != "part_gripper":
            # Determine which tool changer station to use based on part location
            if part_pose.position.y < 0:
                station = "kts1"
            else:
                station = "kts2"
            self.get_logger().info(f"Need to change gripper at {station}")

            # Move to the appropriate tool changer station
            success = self._move_floor_robot_to_joint_position(f"floor_{station}_js_")
            if not success:
                self.get_logger().error("Failed to move to gripper change station")
                return False

            # Perform the gripper change operation
            self._floor_robot_change_gripper(station, "parts")

        # APPROACH PHASE:
        # Move to the bin where the part is located
        success = self._move_floor_robot_to_joint_position(bin_side)
        if not success:
            self.get_logger().error(f"Failed to move to {bin_side}")
            return False

        # Calculate the part rotation for proper gripper alignment
        # We extract the yaw component from the part's orientation
        part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
        # Construct gripper orientation with proper alignment to the part
        gripper_orientation = quaternion_from_euler(0.0, pi, part_rotation)

        # Move to a position above the part for a safe approach
        self.get_logger().info("Moving above the part")
        above_pose = build_pose(
            part_pose.position.x,
            part_pose.position.y,
            part_pose.position.z + 0.3,  # Height offset for safe approach
            gripper_orientation,
        )

        success = self._move_floor_robot_to_pose(above_pose)
        if not success:
            self.get_logger().error("Failed to move above part")
            return False

        # PICKING PHASE:
        # Move down to the part using Cartesian motion for precise movement
        self.get_logger().info("Moving down to part")
        waypoints = [
            build_pose(
                part_pose.position.x,
                part_pose.position.y,
                # Calculate optimal position for gripper contact
                # Use part height plus a small offset to ensure proper contact
                part_pose.position.z
                + RobotController._part_heights[part_to_pick.type]
                + 0.008,  # Small offset for better contact
                gripper_orientation,
            )
        ]
        # Use slower motion for precision with collision checking disabled
        self._move_floor_robot_cartesian(waypoints, 0.1, 0.1, False)
        self.get_logger().info("Moved down to part")

        # Enable the vacuum gripper
        gripper_future = self.set_floor_robot_gripper_state(True)
        if gripper_future:
            # Wait for the gripper service call to complete
            wait_count = 0
            while not gripper_future.done() and wait_count < 20:
                time.sleep(0.1)
                wait_count += 1
            self.get_logger().info("Gripper enabled")

        # Wait for part attachment, with error handling
        try:
            self.get_logger().info("Wait for attach")
            # This function will try multiple small movements if needed
            self._floor_robot_wait_for_attach(30.0, gripper_orientation)
            self.get_logger().info("Part attached successfully")
        except Error as e:
            # Handle attachment failure with safe recovery
            self.get_logger().error(f"Attachment failed: {str(e)}")

            # Move up to a safe position
            waypoints = [
                build_pose(
                    part_pose.position.x,
                    part_pose.position.y,
                    part_pose.position.z + 0.1,  # Safe distance above part
                    gripper_orientation,
                )
            ]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

            # Disable gripper and return to a safe bin position
            self.set_floor_robot_gripper_state(False)
            self._move_floor_robot_to_joint_position(bin_side)
            return False

        # RETREAT PHASE:
        # Move back to bin position with part attached
        self.get_logger().info("Moving back to bin position with part")
        success = self._move_floor_robot_to_joint_position(bin_side)
        if not success:
            self.get_logger().error(f"Failed to move back to {bin_side}")
            # Continue despite this failure - we can still try to complete the operation

        # SCENE UPDATE PHASE:
        # Update the planning scene with the attached part for collision checking
        self._attach_model_to_floor_gripper(part_to_pick, part_pose)

        # Update the robot's internal state to track the attached part
        self.floor_robot_attached_part_ = part_to_pick

        # Entire pick operation completed successfully
        return True

    def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
        """
        Attach a part to the floor gripper in the planning scene.

        Args:
            part_to_pick (PartMsg): Part to attach (contains color and type)
            part_pose (Pose): Pose of the part
        """
        # Create a name for the part based on its color and type
        part_name = (
            self._part_colors[part_to_pick.color]
            + "_"
            + self._part_types[part_to_pick.type]
        )

        self.get_logger().info(f"Attaching {part_name} to floor gripper")

        # Get the path to the mesh file for the part
        model_path = self.mesh_file_path + self._part_types[part_to_pick.type] + ".stl"

        # Create an attached collision object for the part
        attached_collision_object = self._make_attached_mesh(
            part_name, part_pose, model_path, "floor_robot"
        )

        # Create a temporary scene to update
        temp_scene = copy(self.planning_scene_msg)

        # Update the planning scene with the attached part
        with self._planning_scene_monitor.read_write() as scene:
            temp_scene.world.collision_objects = self._world_collision_objects
            temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
            temp_scene.robot_state.attached_collision_objects.append(
                attached_collision_object
            )

            # Apply the updated planning scene
            applied = self.apply_planning_scene(temp_scene)
            if not applied:
                self.get_logger().warn(
                    "Failed to update planning scene with attached part"
                )

            # Update the current state
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

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
