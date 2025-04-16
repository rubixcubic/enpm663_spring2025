from argparse import _MutuallyExclusiveGroup
from distutils.command import build
from time import sleep
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
from rclpy.time import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from std_msgs.msg import Header

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
import cv2
from imutils.object_detection import non_max_suppression
import numpy as np
from ament_index_python import get_package_share_directory
from os import path


from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
    ApplyPlanningScene,
    GetPlanningScene,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    AssemblyState as AssemblyStateMsg,
    CombinedTask as CombinedTaskMsg,
    VacuumGripperState,
)

from ariac_msgs.srv import MoveAGV, VacuumGripperControl, ChangeGripper, SubmitOrder

from std_srvs.srv import Trigger

from ariac_tutorials.utils import (
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


class MoveItDemoInterface(Node):
    """
    Class for demonstrating the different concepts of MoveIt.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    """

    _competition_states = {
        CompetitionStateMsg.IDLE: "idle",
        CompetitionStateMsg.READY: "ready",
        CompetitionStateMsg.STARTED: "started",
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: "order_announcements_done",
        CompetitionStateMsg.ENDED: "ended",
    }
    """Dictionary for converting CompetitionState constants to strings"""

    _part_colors = {
        PartMsg.RED: "red",
        PartMsg.BLUE: "blue",
        PartMsg.GREEN: "green",
        PartMsg.ORANGE: "orange",
        PartMsg.PURPLE: "purple",
    }
    """Dictionary for converting Part color constants to strings"""

    _part_colors_emoji = {
        PartMsg.RED: "🟥",
        PartMsg.BLUE: "🟦",
        PartMsg.GREEN: "🟩",
        PartMsg.ORANGE: "🟧",
        PartMsg.PURPLE: "🟪",
    }
    """Dictionary for converting Part color constants to emojis"""

    _part_types = {
        PartMsg.BATTERY: "battery",
        PartMsg.PUMP: "pump",
        PartMsg.REGULATOR: "regulator",
        PartMsg.SENSOR: "sensor",
    }
    """Dictionary for converting Part type constants to strings"""


    _gripper_states = {True: "enabled", False: "disabled"}
    """Dictionary for converting VacuumGripperState constants to strings"""

    _part_heights = {
        PartMsg.BATTERY: 0.04,
        PartMsg.PUMP: 0.12,
        PartMsg.REGULATOR: 0.07,
        PartMsg.SENSOR: 0.07,
    }
    """Dictionary for the heights of each part"""

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

    def __init__(self, enable_moveit=True):
        super().__init__("moveit_demo_interface")

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # ROS2 callback groups
        self.ariac_cb_group = MutuallyExclusiveCallbackGroup()
        self.moveit_cb_group = MutuallyExclusiveCallbackGroup()

        # Service clients for starting and ending the competition
        self._start_competition_client = self.create_client(
            Trigger, "/ariac/start_competition"
        )
        self._end_competition_client = self.create_client(
            Trigger, "/ariac/end_competition"
        )

        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            "/ariac/competition_state",
            self._competition_state_cb,
            10,
            callback_group=self.ariac_cb_group,
        )

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        # Subscriber to the floor gripper state topic
        self._floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            "/ariac/floor_robot_gripper_state",
            self._floor_robot_gripper_state_cb,
            qos_profile_sensor_data,
            callback_group=self.ariac_cb_group,
        )

        # Service client for turning on/off the vacuum gripper on the floor robot
        self._floor_gripper_enable = self.create_client(
            VacuumGripperControl, "/ariac/floor_robot_enable_gripper"
        )

        # Attribute to store the current state of the floor robot gripper
        self._floor_robot_gripper_state = VacuumGripperState()

        # Moveit_py variables
        self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
        self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())
        self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
        self._floor_robot_home_quaternion = Quaternion()

        self._planning_scene_monitor = (
            self._ariac_robots.get_planning_scene_monitor()
        )

        self._world_collision_objects = []

        # service clients
        self.get_cartesian_path_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )
        self.get_position_fk_client = self.create_client(
            GetPositionFK, "compute_fk"
        )
        
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
        self.mesh_file_path = (
            get_package_share_directory("ariac_tutorials") + "/meshes/"
        )

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

    def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        """Callback for the topic /ariac/floor_robot_gripper_state

        Arguments:
            msg -- VacuumGripperState message
        """
        self._floor_robot_gripper_state = msg

    def start_competition(self):
        """Function to start the competition."""
        self.get_logger().info("Waiting for competition to be ready")

        if self._competition_state == CompetitionStateMsg.STARTED:
            return

        # Wait for competition to be ready
        while self._competition_state != CompetitionStateMsg.READY:
            pass

        self.get_logger().info("Competition is ready. Starting...")

        # Check if service is available
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "Service '/ariac/start_competition' is not available."
            )
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info("Started competition.")
        else:
            self.get_logger().warn("Unable to start competition")

    def end_competition(self):
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

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info("Ended competition.")
        else:
            self.get_logger().warn("Unable to end competition")

    def set_floor_robot_gripper_state(self, state):
        """Set the gripper state of the floor robot.

        Arguments:
            state -- True to enable, False to disable

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f"Gripper is already {self._gripper_states[state]}")
            return

        request = VacuumGripperControl.Request()
        request.enable = state

        future = self._floor_gripper_enable.call_async(request)

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info(
                f"Changed gripper state to {self._gripper_states[state]}"
            )
        else:
            self.get_logger().warn("Unable to change gripper state")

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


        request.waypoints = waypoints
        request.max_step = 0.1
        request.avoid_collisions = avoid_collision
        request.max_velocity_scaling_factor = max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        future = self.get_cartesian_path_client.call_async(request)

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
        """Helper function to plan and execute a motion."""
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
            )
        else:
            logger.error("Planning failed")
            return False
        return True

    def move_floor_robot_home(self):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            self._floor_robot.set_goal_state(configuration_name="home")
        self._plan_and_execute(
            self._ariac_robots,
            self._floor_robot,
            self.get_logger(),
            "floor_robot",
            sleep_time=0.0,
        )
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
            self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose(
                "floor_gripper"
            ).orientation


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

    def _move_floor_robot_to_pose(self, pose: Pose):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose = pose
            self._floor_robot.set_goal_state(
                pose_stamped_msg=pose_goal, pose_link="floor_gripper"
            )

        while not self._plan_and_execute(
            self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"
        ):
            pass

    def _makeMesh(self, name, pose, filename, frame_id) -> CollisionObject:
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

        o = CollisionObject()
        o.header.frame_id = frame_id
        o.id = name
        o.meshes.append(mesh)
        o.mesh_poses.append(pose)
        o.operation = o.ADD
        return o

    def _add_model_to_planning_scene(
        self, name: str, mesh_file: str, model_pose: Pose, frame_id="world"
    ):
        self.get_logger().info(f"Adding {name} to planning scene")
        model_path = self.mesh_file_path + mesh_file
        collision_object = self._makeMesh(
            name, model_pose, model_path, frame_id=frame_id
        )
        with self._planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            self._world_collision_objects.append(collision_object)
            scene.current_state.update()

    def add_objects_to_planning_scene(self):
        package_share_directory = get_package_share_directory("ariac_tutorials")
        with open(
            package_share_directory + "/config/collision_object_info.yaml", "r"
        ) as object_file:
            objects_dict = yaml.safe_load(object_file)

        objects_dict: dict
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


    def load_part_templates(self):
        self.sensor_template = cv2.imread(
            path.join(
                get_package_share_directory("ariac_tutorials"),
                "resources",
                "sensor.png",
            ),
            cv2.IMREAD_GRAYSCALE,
        )
        self.regulator_template = cv2.imread(
            path.join(
                get_package_share_directory("ariac_tutorials"),
                "resources",
                "regulator.png",
            ),
            cv2.IMREAD_GRAYSCALE,
        )
        self.battery_template = cv2.imread(
            path.join(
                get_package_share_directory("ariac_tutorials"),
                "resources",
                "battery.png",
            ),
            cv2.IMREAD_GRAYSCALE,
        )
        self.pump_template = cv2.imread(
            path.join(
                get_package_share_directory("ariac_tutorials"), "resources", "pump.png"
            ),
            cv2.IMREAD_GRAYSCALE,
        )

        if (
            (not self.sensor_template.shape[0] > 0)
            or (not self.regulator_template.shape[0] > 0)
            or (not self.battery_template.shape[0] > 0)
            or (not self.pump_template.shape[0] > 0)
        ):
            return False
        return True


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
            sleep(0.2)
            if time.time() - start_time >= timeout:
                self.get_logger().error("Unable to pick up part")

    def floor_robot_pick_bin_part(self, part_to_pick: PartMsg):
        if not self.moveit_enabled:
            self.get_logger().error("Moveit_py is not enabled, unable to pick bin part")
            return
        part_pose = Pose()
        found_part = False
        bin_side = ""
        for part in self._left_bins_parts:
            part: PartPoseMsg
            if (
                part.part.type == part_to_pick.type
                and part.part.color == part_to_pick.color
            ):
                part_pose = multiply_pose(self._left_bins_camera_pose, part.pose)
                found_part = True
                bin_side = "left_bins"
                break

        if not found_part:
            for part in self._right_bins_parts:
                part: PartPoseMsg
                if (
                    part.part.type == part_to_pick.type
                    and part.part.color == part_to_pick.color
                ):
                    part_pose = multiply_pose(self._right_bins_camera_pose, part.pose)
                    found_part = True
                    bin_side = "right_bins"
                    break

        if not found_part:
            self.get_logger().error("Unable to locate part")
        else:
            self.get_logger().info(f"Part found in {bin_side}")

        if self._floor_robot_gripper_state.type != "part_gripper":
            if part_pose.position.y < 0:
                station = "kts1"
            else:
                station = "kts2"
            self.floor_robot_move_to_joint_position(f"floor_{station}_js_")
            self._floor_robot_change_gripper(station, "parts")

        self.floor_robot_move_to_joint_position(bin_side)
        part_rotation = rpy_from_quaternion(part_pose.orientation)[2]

        gripper_orientation = quaternion_from_euler(0.0, pi, part_rotation)
        self._move_floor_robot_to_pose(
            build_pose(
                part_pose.position.x,
                part_pose.position.y,
                part_pose.position.z + 0.5,
                gripper_orientation,
            )
        )

        waypoints = [
            build_pose(
                part_pose.position.x,
                part_pose.position.y,
                part_pose.position.z
                + CompetitionInterface._part_heights[part_to_pick.type]
                + 0.008,
                gripper_orientation,
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
        self.set_floor_robot_gripper_state(True)
        self._floor_robot_wait_for_attach(30.0, gripper_orientation)

        self.floor_robot_move_to_joint_position(bin_side)

        self._attach_model_to_floor_gripper(part_to_pick, part_pose)

        self.floor_robot_attached_part_ = part_to_pick


    def floor_robot_pick_and_place_tray(self, tray_id, agv_number):
        if not self.moveit_enabled:
            self.get_logger().error("Moveit_py is not enabled, pick and place tray")
            return
        tray_pose = Pose
        station = ""
        found_tray = False

        for tray in self._kts1_trays:
            if tray.id == tray_id:
                station = "kts1"
                tray_pose = multiply_pose(self._kts1_camera_pose, tray.pose)
                found_tray = True
                break

        if not found_tray:
            for tray in self._kts2_trays:
                if tray.id == tray_id:
                    station = "kts2"
                    tray_pose = multiply_pose(self._kts2_camera_pose, tray.pose)
                    found_tray = True
                    break

        if not found_tray:
            return False

        tray_rotation = rpy_from_quaternion(tray_pose.orientation)[2]

        self.floor_robot_move_to_joint_position(f"floor_{station}_js_")

        if self._floor_robot_gripper_state.type != "tray_gripper":
            self._floor_robot_change_gripper(station, "trays")

        gripper_orientation = quaternion_from_euler(0.0, pi, tray_rotation)
        self._move_floor_robot_to_pose(
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + 0.5,
                gripper_orientation,
            )
        )

        waypoints = [
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + 0.003,
                gripper_orientation,
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
        self.set_floor_robot_gripper_state(True)
        self._floor_robot_wait_for_attach(30.0, gripper_orientation)
        waypoints = [
            build_pose(
                tray_pose.position.x,
                tray_pose.position.y,
                tray_pose.position.z + 0.5,
                gripper_orientation,
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)

        # self.floor_robot_move_joints_dict({"linear_actuator_joint":self._rail_positions[f"agv{agv_number}"],
        #                                "floor_shoulder_pan_joint":0})

        agv_tray_pose = self._frame_world_pose(f"agv{agv_number}_tray")
        agv_rotation = rpy_from_quaternion(agv_tray_pose.orientation)[2]

        agv_quaternion = quaternion_from_euler(0.0, pi, agv_rotation)

        self._move_floor_robot_to_pose(
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.5,
                agv_quaternion,
            )
        )

        waypoints = [
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.01,
                agv_quaternion,
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)
        self.set_floor_robot_gripper_state(False)
        self.lock_agv_tray(agv_number)

        waypoints = [
            build_pose(
                agv_tray_pose.position.x,
                agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.3,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]
        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)


    def _floor_robot_place_part_on_kit_tray(self, agv_num: int, quadrant: int):
        if not self._floor_robot_gripper_state.attached:
            self.get_logger().error("No part attached")
            return False

        self.floor_robot_move_to_joint_position(f"agv{agv_num}")

        agv_tray_pose = self._frame_world_pose(f"agv{agv_num}_tray")

        part_drop_offset = build_pose(
            CompetitionInterface._quad_offsets[quadrant][0],
            CompetitionInterface._quad_offsets[quadrant][1],
            0.0,
            quaternion_from_euler(0.0, pi, 0.0),
        )

        part_drop_pose = multiply_pose(agv_tray_pose, part_drop_offset)

        self._move_floor_robot_to_pose(
            build_pose(
                part_drop_pose.position.x,
                part_drop_pose.position.y,
                part_drop_pose.position.z + 0.3,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        )

        waypoints = [
            build_pose(
                part_drop_pose.position.x,
                part_drop_pose.position.y,
                part_drop_pose.position.z
                + CompetitionInterface._part_heights[
                    self.floor_robot_attached_part_.type
                ]
                + 0.01,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

        self.set_floor_robot_gripper_state(False)

        self._remove_model_from_floor_gripper()

        waypoints = [
            build_pose(
                part_drop_pose.position.x,
                part_drop_pose.position.y,
                part_drop_pose.position.z + 0.3,
                quaternion_from_euler(0.0, pi, 0.0),
            )
        ]

        self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

        return True


    def _makeAttachedMesh(self, name, pose, filename, robot) -> AttachedCollisionObject:
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
        while not future.done():
            pass

        # Check the result of the service call.
        if future.result().success:
            self.get_logger().info("Succssefully applied new planning scene")
        else:
            self.get_logger().warn(future.result().message)

    def get_planning_scene_msg(self, msg: PlanningScene) -> PlanningScene:
        self.planning_scene_msg = msg

    def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
        part_name = (
            self._part_colors[part_to_pick.color]
            + "_"
            + self._part_types[part_to_pick.type]
        )

        self.get_logger().info(f"Attaching {part_name} to floor gripper")
        model_path = self.mesh_file_path + self._part_types[part_to_pick.type] + ".stl"
        attached_collision_object = self._makeAttachedMesh(
            part_name, part_pose, model_path, "floor_robot"
        )
        temp_scene = copy(self.planning_scene_msg)
        with self._planning_scene_monitor.read_write() as scene:
            temp_scene.world.collision_objects = self._world_collision_objects
            temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
            temp_scene.robot_state.attached_collision_objects.append(
                attached_collision_object
            )
            self.apply_planning_scene(temp_scene)
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

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

    def _create_floor_joint_position_state(self, joint_positions: list) -> dict:
        return {
            "linear_actuator_joint": joint_positions[0],
            "floor_shoulder_pan_joint": joint_positions[1],
            "floor_shoulder_lift_joint": joint_positions[2],
            "floor_elbow_joint": joint_positions[3],
            "floor_wrist_1_joint": joint_positions[4],
            "floor_wrist_2_joint": joint_positions[5],
            "floor_wrist_3_joint": joint_positions[6],
        }

    def _create_floor_joint_position_dict(self, dict_positions={}):
        with self._planning_scene_monitor.read_write() as scene:
            current_positions = scene.current_state.get_joint_group_positions(
                "floor_robot"
            )
            current_position_dict = self._create_floor_joint_position_state(
                current_positions
            )
            for key in dict_positions.keys():
                current_position_dict[key] = dict_positions[key]
        return current_position_dict

    def floor_robot_move_joints_dict(self, dict_positions: dict):
        new_joint_position = self._create_floor_joint_position_dict(dict_positions)
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            scene.current_state.joint_positions = new_joint_position
            joint_constraint = construct_joint_constraint(
                robot_state=scene.current_state,
                joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group(
                    "floor_robot"
                ),
            )
            self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
        self._plan_and_execute(
            self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"
        )

    def floor_robot_move_to_joint_position(self, position_name: str):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            scene.current_state.joint_positions = self.floor_position_dict[
                position_name
            ]
            joint_constraint = construct_joint_constraint(
                robot_state=scene.current_state,
                joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group(
                    "floor_robot"
                ),
            )
            self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
        self._plan_and_execute(
            self._ariac_robots,
            self._floor_robot,
            self.get_logger(),
            robot_type="floor_robot",
        )
