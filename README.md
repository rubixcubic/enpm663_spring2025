# Lecture 9 - Motion Planning

```bash
cd <workspace>
rosdep install --from-paths src -y --ignore-src
pip install -r <path to requirements.txt>
colcon build 
source install/setup.bash
```

Start the environment:
```bash
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=moveit_demo trial_name:=kitting dev_mode:=True
```
Start the demo:

```bash
ros2 launch moveit_demo controller_demo.launch.py rviz:=true
```
The floor robot should pick up a purple pump.


#   MoveIt: A Comprehensive Overview

MoveIt is a powerful open-source framework for motion planning, manipulation, and control of robots. It provides tools for:

* Motion Planning: Calculating collision-free paths for robots to move.
* Manipulation: Handling tasks like grasping and placing objects.
* Perception: Integrating sensor data to understand the environment.
* Control: Executing planned motions on real or simulated robots.

##   MoveIt Modules Used in Python

The primary MoveIt modules used in the provided Python script are:

* `moveit_py`:
    * This is the main Python interface to MoveIt.
    * It provides classes and functions to interact with MoveIt's core functionalities.
    * It's used to create the `MoveItPy` instance, which is the central point for interacting with MoveIt.
* `PlanningSceneMonitor`:
    * This module allows the script to keep track of the robot's environment.
    * It's crucial for collision avoidance.
* `moveit.core.robot_trajectory`:
    * This module is used to represent and manipulate robot trajectories (planned paths of robot joints).
* `moveit.core.robot_state`:
    * This module represents the robot's configuration (joint positions, etc.).
* `moveit.core.kinematic_constraints`:
    * This module helps in defining constraints for motion planning, such as joint position goals.

##   MoveIt Functions and Classes Explained

Here's a breakdown of the key MoveIt-related functions and classes used in the script:

###   1.  Initialization and Setup

* `MoveItPy(node_name="ariac_robots_moveit_py")`:
    * Creates the main `MoveItPy` object.
    * `node_name` is the ROS 2 node name for MoveIt.
* `PlanningSceneMonitor()`:
    * Instantiated as `self._planning_scene_monitor`.
    * Maintains the current state of the planning scene:
        * Robot's configuration.
        * Positions of obstacles/objects.
    * Allows the robot to "perceive" its world for collision avoidance.
* `RobotState(self._ariac_robots.get_robot_model())`:
    * Represents the state of the robot (joint positions).
    * `self._ariac_robots.get_robot_model()` gets the robot's model from MoveIt.
* `self._ariac_robots.get_planning_component("floor_robot")`:
    * Obtains a `PlanningComponent` for the "floor_robot".
    * A `PlanningComponent` plans motions for a specific robot/joint group.

###   2.  Motion Planning

* `self._floor_robot.set_goal_state(...)`:
    * Defines the goal for motion planning.
    * Goal can be:
        * `pose_stamped_msg`: Desired Cartesian pose for the end effector.
        * `motion_plan_constraints`: Constraints on joint positions.
* `construct_joint_constraint(...)`:
    * (From `moveit.core.kinematic_constraints`)
    * Creates a constraint specifying desired joint positions.
* `self._floor_robot.plan()`:
    * The core motion planning function.
    * Calculates a collision-free trajectory to the goal state.
* `self._ariac_robots.execute(trajectory, controllers=[...])`:
    * Executes the planned `trajectory` on the robot using controllers.

###   3.  Robot Trajectories

* `RobotTrajectory(self._ariac_robots.get_robot_model())`:
    * Represents a planned sequence of robot states (joint positions) over time.
* `trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)`:
    * Sets the trajectory message into the `RobotTrajectory` object.
* `trajectory.joint_model_group_name = "floor_robot"`:
    * Specifies the robot joint group for the trajectory.

###   4.  Cartesian Path Planning

* `self._call_get_cartesian_path(...)`:
    * Calls a service (`compute_cartesian_path`) to plan a Cartesian path (end-effector path in 3D space).
* `GetCartesianPath.Request()`:
    * Service request message for Cartesian path computation.
* `GetCartesianPath.Response`:
    * Service response, containing the planned Cartesian trajectory.

###   5.  Planning Scene Management

* `CollisionObject()`:
    * Represents an object in the environment that can collide with the robot.
    * Used for obstacles (bins, tables).
* `AttachedCollisionObject()`:
    * Represents an object attached to the robot (part held by gripper).
* `PlanningScene()`:
    * Message representing the entire state of the planning scene.
    * Used to communicate environment changes to MoveIt.
* `self._apply_planning_scene(scene)`:
    * Sends a `PlanningScene` message to update the environment.
* `self._planning_scene_monitor.read_write() as scene:`
    * Context manager for thread-safe access to the planning scene.
* `scene.apply_collision_object(collision_object)`:
    * Adds a `CollisionObject` to the planning scene.
* `scene.current_state.update()`:
    * Updates the internal robot state.
* `scene.attachBody()` / `scene.detachObject()`:
    * Attaches/detaches `CollisionObject` to/from the robot.

###   Important Notes

* **Asynchronous ROS 2 Services:**
    * The script uses services (`/apply_planning_scene`, `compute_cartesian_path`).
    * `call_async()` is used for asynchronous calls (non-blocking).
    * Callbacks handle service responses.
* **Planning Scene Updates:**
    * Accurate planning scene updates are crucial for MoveIt's awareness of the environment.
    * `PlanningScene` messages and `PlanningSceneMonitor` are used.
* **Collision Avoidance:**
    * MoveIt's core function is collision avoidance.
    * The planning scene enables this.

##   Motion Plan Constraints: A Deeper Dive

`motion_plan_constraints` allow specifying restrictions/requirements on robot motion during planning.

###   Types of Constraints

* **Joint Constraints:**
    * Specify desired positions for robot joints.
    * Used for precise robot configurations.

###   `construct_joint_constraint`

* **Input:**
    * `robot_state`: A `RobotState` with desired joint configuration.
    * `joint_model_group`: Which joint group the constraint applies to.
* **Output:**
    * A `JointConstraint` object.
* **Usage:**
    * Create `RobotState`, set `joint_positions`.
    * Use `construct_joint_constraint` to create the constraint.
    * Provide the constraint to `set_goal_state`.

###   Why Use Joint Constraints?

* Precise positioning
* Avoiding singularities
* Task requirements
* Reproducibility

###   Example from Your Code (`_move_floor_robot_to_joint_position`)

* The code wants the robot to move to a predefined joint configuration (like "home" or a station position).
* It creates a `RobotState` (`goal_state`) and sets the `joint_positions` to the desired values.
* `construct_joint_constraint` turns this `goal_state` into a `JointConstraint`.
* This constraint is then given to `set_goal_state`, telling MoveIt to plan a motion that ends with the robot's joints at those specific positions.

##   Planning Scene: Detailed Explanation

The planning scene is a central concept in MoveIt. It represents the robot's world and is used for motion planning, collision checking, and other operations.

###   Key Components of the Planning Scene

* **Robot Model:**
    * A description of the robot's kinematics and dynamics (e.g., links, joints, degrees of freedom).
    * MoveIt uses this to calculate robot poses and motions.
* **Robot State:**
    * The current configuration of the robot, including joint positions/velocities/accelerations.
    * The planning scene maintains the robot's current and goal states.
* **Collision Objects:**
    * Represent static objects in the environment that the robot can collide with (e.g., tables, walls, bins).
    * These objects are defined by their shapes (e.g., boxes, cylinders, meshes) and poses (position and orientation).
* **Attached Collision Objects:**
    * Represent objects that are temporarily attached to the robot (e.g., a part held by the gripper).
    * Their pose is defined relative to the robot's link they are attached to.
* **Transforms:**
    * Coordinate transformations between different frames in the scene.
    * Essential for calculating object poses and planning motions.

###   PlanningSceneMonitor

* The `PlanningSceneMonitor` is a crucial class for managing the planning scene.
* It provides an interface for:
    * Getting the current planning scene.
    * Updating the planning scene (adding/removing objects, changing robot state).
    * Publishing the planning scene to other ROS 2 nodes (e.g., RViz for visualization).
* It maintains a thread-safe representation of the planning scene, ensuring consistency.

###   Updating the Planning Scene

* The planning scene needs to be updated whenever the environment changes.
* Common update operations include:
    * Adding `CollisionObject`s for new obstacles.
    * Removing `CollisionObject`s for removed obstacles.
    * Attaching/detaching `AttachedCollisionObject`s when the robot grasps/releases objects.
    * Updating the robot's `RobotState` as it moves.
* In ROS 2, the `PlanningScene` message is used to communicate these updates.

###   Importance of the Planning Scene

* **Collision Avoidance:** The planning scene is essential for collision checking during motion planning. MoveIt uses it to ensure that the planned paths are collision-free.
* **Accurate Motion Planning:** The planning scene provides the planner with a complete representation of the robot and its environment, enabling it to generate accurate and feasible motions.
* **Robot Awareness:** The planning scene allows the robot to "be aware" of its surroundings, which is crucial for complex manipulation tasks.
* **Visualization:** The planning scene can be visualized in RViz, providing a valuable tool for debugging and understanding robot behavior.


<!-- ## Relevant Methods 

### 1. `_control_cb`

This method runs in a timer to orchestrate the whole node. The following lines have been commented out. Uncommenting them will make the robot reach different pre-defined locations using joint-space programming.

#### Join-Space Programming
```python
# Go to different locations using joint-space programming
# self._move_floor_robot_to_joint_position('left_bins')
# self._move_floor_robot_to_joint_position('right_bins')
# self._move_floor_robot_to_joint_position("agv1")
# self._move_floor_robot_to_joint_position("agv2")
# self._move_floor_robot_to_joint_position("agv3")
# self._move_floor_robot_to_joint_position("agv4")
```

#### Picking Up a Part

Initially, the code specifies the target part for the robot (a purple pump). Subsequently, it commands the floor robot to find and pick up this designated part. Because the `_control_cb` function is executed periodically by a timer, a state machine pattern—implemented with the conditions `if not self._part_already_picked_up` and the assignment `self._part_already_picked_up = True`—is used. This prevents the robot from repeatedly attempting to pick up another purple pump in subsequent executions of the timer callback.

```python
# Pick up a part from a bin
if not self._part_already_picked_up:
    part_to_pick = PartMsg()
    part_to_pick.color = PartMsg.PURPLE
    part_to_pick.type = PartMsg.PUMP

    success = self._floor_robot_pick_bin_part(part_to_pick)
    
    if success:
        self._part_already_picked_up = True
        self.get_logger().info("Successfully picked up part")
        # Force refresh the planning scene visualization
        self._refresh_planning_scene_display()
    else:
        self.get_logger().warn("Failed to pick up part, will retry later")
```

### 1. `_move_floor_robot_cartesian`

This method moves the floor robot along a Cartesian path with specific velocity and acceleration controls.

#### What it does:
- Moves the robot's end effector along a straight-line path in Cartesian (x,y,z) space
- Takes a list of waypoints that define the path
- Controls velocity and acceleration scaling factors
- Has an option to avoid collisions during movement
- Uses the ROS GetCartesianPath service to calculate the trajectory

#### Key implementation details:
- Sets minimum velocity and acceleration scaling for faster movement (at least 0.5)
- Calls `_call_get_cartesian_path` to compute the path trajectory
- Converts the trajectory message to a RobotTrajectory object
- Executes the trajectory through the MoveIt framework

#### When to use:
- When you need precise control of the end effector's path
- For small, controlled movements like approaching parts or fixtures
- When you need to ensure a linear motion in 3D space
- For delicate operations where the exact path matters, not just the final position

### 2. `_move_floor_robot_to_joint_position`

This method moves the floor robot to a predefined joint configuration using joint space programming.

#### What it does:
- Moves to predefined positions stored in the `_floor_position_dict`
- Takes a position name (like "left_bins", "right_bins", "agv1", etc.)
- Uses MoveIt's planning framework to plan a path in joint space
- Creates joint constraints to define the goal configuration

#### Key implementation details:
- Sets the start state to the current robot state
- Creates a goal state based on the named joint position
- Uses `construct_joint_constraint` to create a joint constraint for the planner
- Calls `_plan_and_execute` to plan and execute the trajectory

#### When to use:
- For moving to standard, predefined locations
- For large movements across the workspace
- When the exact path doesn't matter, only the final configuration
- When you want to ensure a valid, collision-free path from anywhere to a known position

### 3. `_move_floor_robot_to_pose`

This method moves the floor robot's end effector to a specific pose (position and orientation) in Cartesian space.

#### What it does:
- Takes a target pose (position and orientation) in Cartesian space
- Uses MoveIt's planning framework to plan a path to that pose
- Includes retry logic if planning fails
- Sets the goal for the end effector ("floor_gripper" link)

#### Key implementation details:
- Sets the start state from the current planning scene
- Creates a PoseStamped message with the target pose
- Sets the goal state using the pose for the "floor_gripper" link
- Attempts planning up to 3 times if it fails

#### When to use:
- When you need to move to a specific position and orientation in space
- When you don't care about the exact path, just the final pose
- For positioning the gripper at specific locations, like above a part
- When you have a target in Cartesian coordinates rather than joint angles

### Comparison of the Three Methods

| Method | Space | Control | Best For | Limitations |
|--------|-------|---------|----------|-------------|
| `_move_floor_robot_cartesian` | Cartesian | Path-focused | Precise movements, straight-line motion | May fail if straight-line path is impossible |
| `_move_floor_robot_to_joint_position` | Joint | Configuration-focused | Known positions, large movements | Less precise end effector control |
| `_move_floor_robot_to_pose` | Cartesian | Goal-focused | Specific end effector poses | No control over the path taken |

In practice, a typical robot operation might use all three methods in sequence:
1. Use `_move_floor_robot_to_joint_position` to get to the general area (e.g., "left_bins")
2. Use `_move_floor_robot_to_pose` to position the end effector above a target
3. Use `_move_floor_robot_cartesian` for the final approach and precision movements

This combination gives you both efficiency for large movements and precision for critical operations.

## Understanding Gripper Orientation Calculation

When picking up parts in a robotic system, proper gripper orientation is critical for successful grasping. The code snippet:

```python
part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
gripper_orientation = quaternion_from_euler(0.0, pi, part_rotation)
```

performs two essential operations:

### 1. Extracting the Yaw Angle

`rpy_from_quaternion(part_pose.orientation)[2]` converts the part's quaternion orientation to roll-pitch-yaw angles and extracts just the yaw component (index 2). This gives us the part's rotation around the z-axis.

### 2. Creating the Proper Gripper Orientation

`quaternion_from_euler(0.0, pi, part_rotation)` builds a new quaternion with:
* Roll = 0.0 (no rotation around x-axis)
* Pitch = pi (180° rotation around y-axis, pointing the gripper downward)
* Yaw = part_rotation (matching the part's rotation around z-axis)

### Why This Matters

This calculation ensures that:

1. The gripper approaches from above (pitch = pi)
2. The gripper aligns with the part's orientation (matching the yaw)
3. The gripper maintains a vertical approach vector for stable grasping

Without this orientation matching, the gripper might approach at an angle that prevents proper contact with the part surfaces, leading to failed pick attempts or unstable grasps that could drop the part during movement.

## Planning Scene in Robot Control

### Overview

The planning scene is a crucial component in robotic control systems that represents the robot's understanding of its environment. It serves as a virtual model of the physical world where the robot operates, including obstacles, objects of interest, and the robot itself. This environment model enables safe motion planning and collision avoidance.

### Key Components

#### Planning Scene Monitor

```python
self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()
```

The planning scene monitor provides an interface to access and modify the planning scene. It allows the robot controller to:

- Read the current state of the environment
- Make updates to the scene as the environment changes
- Monitor collisions and constraints in real-time

#### Collision Objects
Collision objects represent physical entities in the robot's workspace:

- **Static environment**: Fixed objects like bins, tables, walls
- **Dynamic objects**: Movable items like parts on a conveyor
- **Robot links**: The robot's own body parts that need collision checking
- **Attached objects**: Items being manipulated by the robot

#### Scene Management Functions

##### Adding Objects

```python
def _add_objects_to_planning_scene(self):
    # Loads objects from YAML configuration
    # Adds them to the scene in batches
```

This function populates the planning scene with collision objects defined in configuration files, typically loading mesh models that represent the physical shapes of objects.


#### Creating Collision Objects

```python
def _create_mesh_collision_object(self, name, mesh_path, pose, frame_id="world"):
    # Creates collision objects from mesh files
```

This method constructs collision objects from 3D mesh files (.stl format), defining their geometry and position in the environment.


#### Attaching Objects

```python
def _attach_model_to_floor_gripper(self, part_to_pick, part_pose):
    # Attaches picked parts to the gripper in the planning scene
```

When a robot grasps an object, this function updates the planning scene to attach the object to the robot's end effector, ensuring collision checking accounts for the grasped item.

##### Scene Verification and Visualization

```python
def _verify_planning_scene_ready(self):
    # Checks if expected objects are present in the scene
```

This function ensures the planning scene is properly initialized with all necessary objects before operations begin.

```python
def _publish_planning_scene_for_rviz(self):
    # Ensures RViz displays an up-to-date representation
```

Visualization tools like RViz display the planning scene to operators, helping with debugging and monitoring. This function ensures the displayed scene is current.

### Applications in Motion Planning
The planning scene enables several critical capabilities:

- Collision detection: Preventing the robot from colliding with objects or itself
- Path planning: Finding feasible, collision-free paths through the environment
- Manipulation planning: Planning complex interactions with objects
- Dynamic updates: Adapting to changes in the environment during operation

### Implementation Details
The planning scene is implemented using MoveIt, a popular motion planning framework for ROS (Robot Operating System). Key implementation aspects include:

- Scene representation: Using occupancy grids, octrees, or collision objects
- Efficient updates: Batching changes to avoid computational overhead
- Frame transformations: Maintaining proper coordinate frames for all objects
- Threading considerations: Safely accessing the scene from multiple processes

By maintaining an accurate planning scene, robots can operate safely and effectively in complex environments while handling various objects and avoiding collisions. -->