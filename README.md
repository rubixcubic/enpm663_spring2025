# Lecture 9 - Robot Movement Methods

## 1. `_move_floor_robot_cartesian`

This method moves the floor robot along a Cartesian path with specific velocity and acceleration controls.

### What it does:
- Moves the robot's end effector along a straight-line path in Cartesian (x,y,z) space
- Takes a list of waypoints that define the path
- Controls velocity and acceleration scaling factors
- Has an option to avoid collisions during movement
- Uses the ROS GetCartesianPath service to calculate the trajectory

### Key implementation details:
- Sets minimum velocity and acceleration scaling for faster movement (at least 0.5)
- Calls `_call_get_cartesian_path` to compute the path trajectory
- Converts the trajectory message to a RobotTrajectory object
- Executes the trajectory through the MoveIt framework

### When to use:
- When you need precise control of the end effector's path
- For small, controlled movements like approaching parts or fixtures
- When you need to ensure a linear motion in 3D space
- For delicate operations where the exact path matters, not just the final position

## 2. `_move_floor_robot_to_joint_position`

This method moves the floor robot to a predefined joint configuration using joint space programming.

### What it does:
- Moves to predefined positions stored in the `_floor_position_dict`
- Takes a position name (like "left_bins", "right_bins", "agv1", etc.)
- Uses MoveIt's planning framework to plan a path in joint space
- Creates joint constraints to define the goal configuration

### Key implementation details:
- Sets the start state to the current robot state
- Creates a goal state based on the named joint position
- Uses `construct_joint_constraint` to create a joint constraint for the planner
- Calls `_plan_and_execute` to plan and execute the trajectory

### When to use:
- For moving to standard, predefined locations
- For large movements across the workspace
- When the exact path doesn't matter, only the final configuration
- When you want to ensure a valid, collision-free path from anywhere to a known position

## 3. `_move_floor_robot_to_pose`

This method moves the floor robot's end effector to a specific pose (position and orientation) in Cartesian space.

### What it does:
- Takes a target pose (position and orientation) in Cartesian space
- Uses MoveIt's planning framework to plan a path to that pose
- Includes retry logic if planning fails
- Sets the goal for the end effector ("floor_gripper" link)

### Key implementation details:
- Sets the start state from the current planning scene
- Creates a PoseStamped message with the target pose
- Sets the goal state using the pose for the "floor_gripper" link
- Attempts planning up to 3 times if it fails

### When to use:
- When you need to move to a specific position and orientation in space
- When you don't care about the exact path, just the final pose
- For positioning the gripper at specific locations, like above a part
- When you have a target in Cartesian coordinates rather than joint angles

## Comparison of the Three Methods

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

# Understanding Gripper Orientation Calculation

When picking up parts in a robotic system, proper gripper orientation is critical for successful grasping. The code snippet:

```python
part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
gripper_orientation = quaternion_from_euler(0.0, pi, part_rotation)
```

performs two essential operations:

## 1. Extracting the Yaw Angle

`rpy_from_quaternion(part_pose.orientation)[2]` converts the part's quaternion orientation to roll-pitch-yaw angles and extracts just the yaw component (index 2). This gives us the part's rotation around the z-axis.

## 2. Creating the Proper Gripper Orientation

`quaternion_from_euler(0.0, pi, part_rotation)` builds a new quaternion with:
* Roll = 0.0 (no rotation around x-axis)
* Pitch = pi (180° rotation around y-axis, pointing the gripper downward)
* Yaw = part_rotation (matching the part's rotation around z-axis)

## Why This Matters

This calculation ensures that:

1. The gripper approaches from above (pitch = pi)
2. The gripper aligns with the part's orientation (matching the yaw)
3. The gripper maintains a vertical approach vector for stable grasping

Without this orientation matching, the gripper might approach at an angle that prevents proper contact with the part surfaces, leading to failed pick attempts or unstable grasps that could drop the part during movement.

# Planning Scene in Robot Control

## Overview

The planning scene is a crucial component in robotic control systems that represents the robot's understanding of its environment. It serves as a virtual model of the physical world where the robot operates, including obstacles, objects of interest, and the robot itself. This environment model enables safe motion planning and collision avoidance.

## Key Components

### Planning Scene Monitor

```python
self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()
```

The planning scene monitor provides an interface to access and modify the planning scene. It allows the robot controller to:

- Read the current state of the environment
- Make updates to the scene as the environment changes
- Monitor collisions and constraints in real-time

### Collision Objects
Collision objects represent physical entities in the robot's workspace:

- **Static environment**: Fixed objects like bins, tables, walls
- **Dynamic objects**: Movable items like parts on a conveyor
- **Robot links**: The robot's own body parts that need collision checking
- **Attached objects**: Items being manipulated by the robot

### Scene Management Functions

#### Adding Objects

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

#### Scene Verification and Visualization

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

## Applications in Motion Planning
The planning scene enables several critical capabilities:

- Collision detection: Preventing the robot from colliding with objects or itself
- Path planning: Finding feasible, collision-free paths through the environment
- Manipulation planning: Planning complex interactions with objects
- Dynamic updates: Adapting to changes in the environment during operation

## Implementation Details
The planning scene is implemented using MoveIt, a popular motion planning framework for ROS (Robot Operating System). Key implementation aspects include:

- Scene representation: Using occupancy grids, octrees, or collision objects
- Efficient updates: Batching changes to avoid computational overhead
- Frame transformations: Maintaining proper coordinate frames for all objects
- Threading considerations: Safely accessing the scene from multiple processes

By maintaining an accurate planning scene, robots can operate safely and effectively in complex environments while handling various objects and avoiding collisions.