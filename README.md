# Lecture 9 - Motion Planning

## Build
```bash
cd <workspace>
rosdep install --from-paths src -y --ignore-src
pip install -r <path to requirements.txt>
colcon build 
source install/setup.bash
```

## Start the environment
```bash
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=moveit_demo trial_name:=kitting dev_mode:=True record_state:=false
```
## Start the demo (choose one):
- Part pick-and-place (Python)
```bash
ros2 launch moveit_demo controller_demo.launch.py program:=python rviz:=true operation_mode:=pick_place_part
```
- Part pick-and-place (C++)
```bash
ros2 launch moveit_demo controller_demo.launch.py program:=cpp rviz:=true operation_mode:=pick_place_part
```
- Tray pick-and-place (Python)
```bash
ros2 launch moveit_demo controller_demo.launch.py program:=python rviz:=true operation_mode:=pick_place_tray
```
- Tray pick-and-place (C++)
```bash
ros2 launch moveit_demo controller_demo.launch.py program:=cpp rviz:=true operation_mode:=pick_place_tray
```

## Pipeline ```minimal_demo.py```

![Alt text](./lecture9/moveit_demo/figures/minimal_demo_py.png)


#   MoveIt: A Comprehensive Overview

MoveIt is a powerful open-source framework for motion planning, manipulation, and control of robots. It provides tools for:

* Motion Planning: Calculating collision-free paths for robots to move.
* Manipulation: Handling tasks like grasping and placing objects.
* Perception: Integrating sensor data to understand the environment.
* Control: Executing planned motions on real or simulated robots.

# MoveIt Interfaces in C++ and Python: A Comparison

This document compares the implementation of robot motion planning and control using MoveIt in both Python and C++. Based on the provided code samples, we'll explore key differences, similarities, and patterns in both languages.

## Core MoveIt Class Structure

### Python Implementation

* Uses the `moveit_py` module as the primary interface
* Creates a `MoveItPy` instance as the central point for interacting with MoveIt
* Uses `PlanningSceneMonitor` for managing the planning scene 
* Leverages `RobotState` and `RobotTrajectory` classes from `moveit.core`

```python
self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()
self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())
self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
```

### C++ Implementation

* Uses `MoveGroupInterface` class as the primary interface
* Creates separate interfaces for different robot groups
* Uses `PlanningSceneInterface` for managing the planning scene
* Leverages `TimeOptimalTrajectoryGeneration` for trajectory optimization

```cpp
moveit::planning_interface::MoveGroupInterface floor_robot_;
moveit::planning_interface::MoveGroupInterface ceiling_robot_;
moveit::planning_interface::PlanningSceneInterface planning_scene_;
trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
```

```floor_robot``` and ```ceiling_robot``` objects are initialized in the constructor:

```cpp
RobotController::RobotController()
    : Node("moveit_demo"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
}
```

## Planning Scene Management

### Python Implementation

* Uses context managers for thread-safe access to the planning scene
* Directly applies collision objects to the scene
* Updates the planning scene using monitor's read_write method

```python
with self._planning_scene_monitor.read_write() as scene:
    # Apply the collision object
    scene.apply_collision_object(collision_object)
    # Update the scene
    scene.current_state.update()
```

### C++ Implementation

* Uses direct calls to the planning scene interface
* Applies collision objects using the planning scene interface
* Uses a separate service to apply planning scene updates

```cpp
planning_scene_.applyCollisionObject(CreateCollisionObject(name, mesh_file, model_pose));
```

## Motion Planning and Execution

### Python Implementation

* Sets goals using constraints or pose targets
* Plans and executes as separate steps
* Uses callbacks for asynchronous service calls
* Provides detailed error recovery mechanisms

```python
# Set goal
self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
# Plan and execute
success = self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot")
```

### C++ Implementation

* Sets goals using named targets or joint values
* Plans and executes as separate steps with explicit plan object
* Provides retry mechanisms with adaptive parameters
* Uses a time-optimal trajectory generation post-processor

```cpp
// Set goal
floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
// Plan
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = static_cast<bool>(floor_robot_.plan(plan));
// Execute
if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
}
```

## Cartesian Path Planning

### Python Implementation

* Uses a service client for Cartesian path computation
* Handles service responses asynchronously
* Converts trajectory messages to robot trajectory objects

```python
trajectory_msg = self._call_get_cartesian_path(waypoints, velocity, acceleration, avoid_collision, "floor_robot")
trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
trajectory.set_robot_trajectory_msg(scene.current_state, trajectory_msg)
trajectory.joint_model_group_name = "floor_robot"
self._ariac_robots.execute(trajectory, controllers=[])
```

### C++ Implementation

* Directly calls computeCartesianPath method on the move group interface
* Post-processes trajectories with time parameterization
* Implements segment-by-segment fallback for complex paths

```cpp
double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);
// Retime trajectory
robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
totg_.computeTimeStamps(rt, vsf, asf);
rt.getRobotTrajectoryMsg(trajectory);
return static_cast<bool>(floor_robot_.execute(trajectory));
```

## Error Handling and Recovery

### Python Implementation

* Uses extensive logging for error tracking
* Implements timeouts for service calls
* Provides fallback mechanisms for planning scene updates
* Uses try-except blocks for error handling

```python
try:
    # Attempt to get the result of the service call
    result = future.result()
    # Check the 'success' field of the service response
    if result.success:
        self.get_logger().info("Started competition.")
    else:
        self.get_logger().warn("Unable to start competition")
except Exception as e:
    self.get_logger().error(f"Competition start service call failed: {e}")
```

### C++ Implementation

* Uses multiple planning attempts with increasing planning time
* Adjusts planning parameters between attempts
* Tries alternative planners if the default fails
* Implements segment-by-segment planning as a fallback

```cpp
int max_attempts = 3;
for (int attempt = 1; attempt <= max_attempts; attempt++) {
    // Use more planning time for each successive attempt
    floor_robot_.setPlanningTime(5.0 + (attempt * 5.0));
    bool success = static_cast<bool>(floor_robot_.plan(plan));
    
    if (success) {
        // Execute plan
    } else {
        // Try adjusting planning parameters
        if (attempt < max_attempts) {
            floor_robot_.setMaxVelocityScalingFactor(0.8 - (0.2 * attempt));
            floor_robot_.setMaxAccelerationScalingFactor(0.8 - (0.2 * attempt));
            // Try a different planner
            if (attempt == 2) {
                floor_robot_.setPlannerId("RRTConnect");
            } else if (attempt == 3) {
                floor_robot_.setPlannerId("BiTRRT");
            }
        }
    }
}
```

## Advantages of Each Implementation

### Python Advantages

* More expressive context managers for thread-safety
* Cleaner callback handling for asynchronous operations
* More intuitive object model for planning scene manipulation
* Better built-in service client abstractions

### C++ Advantages

* More direct control over planning parameters
* Built-in trajectory optimization
* More efficient memory management
* Better performance for computationally intensive operations
* More explicit error handling with return values

## Best Practices From Both Implementations

1. **Implement retry mechanisms with adaptive parameters**
2. **Use thread-safe access to the planning scene**
3. **Provide fallbacks for different planning methods**
4. **Validate inputs and poses before planning**
5. **Use appropriate move strategies for different situations**
6. **Implement staged approaches for critical operations**
7. **Add collision objects to planning scene for safety**
8. **Validate operation success at each step**

## Key Differences in Approach

1. Python uses more asynchronous patterns with callbacks
2. C++ implements more sophisticated retry mechanisms
3. Python leverages context managers for thread safety
4. C++ provides more direct control over planning parameters
5. Python has more concise syntax for common operations
6. C++ has more explicit error handling with return values
7. Python uses service clients for various operations
8. C++ directly calls methods on the move group interface

# Advanced MoveIt Concepts: Constraints and Planning Scene in Python and C++

## Motion Plan Constraints

Motion plan constraints are powerful tools in MoveIt that allow developers to specify precise requirements for robot motion during planning. Rather than simply specifying an end position, constraints give you fine-grained control over how the robot should move and what configurations are acceptable.

### Understanding Joint Constraints

Joint constraints are the most commonly used type of constraint in MoveIt. They allow you to:

- Specify exact positions for specific robot joints
- Define acceptable ranges for joint values
- Ensure the robot achieves a particular configuration
- Avoid problematic poses or singularities

### Creating and Using Joint Constraints

#### Python Implementation

In Python, the `construct_joint_constraint` function is the primary method for creating joint constraints:

```python
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
    joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group("floor_robot")
)

# Set goal using the constraint
self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
```

#### C++ Implementation

In C++, joint constraints are typically implemented by directly setting joint target values on the MoveGroupInterface:

```cpp
// Set specific joint values for a target position
floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

// Alternative: set multiple joint values at once using a map
std::map<std::string, double> joint_positions = {
    {"linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]},
    {"floor_shoulder_pan_joint", 0.0},
    {"floor_shoulder_lift_joint", -1.0},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
};
floor_robot_.setJointValueTarget(joint_positions);

// Or use predefined named targets
floor_robot_.setNamedTarget("home");
```

### Benefits of Constraint-Based Planning

Using constraints for motion planning offers several advantages:

- **Precision**: Achieve exact joint configurations when needed
- **Safety**: Avoid dangerous robot configurations
- **Task-specific positioning**: Match robot pose to task requirements
- **Reproducibility**: Ensure consistent robot behavior across executions
- **Complex motion control**: Define sophisticated movement patterns

### Practical Application Examples

#### Python Example: Moving to a Specific Joint Position

```python
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

        return success
    except Exception as e:
        self.get_logger().error(f"Error: {str(e)}")
        return False
```

#### C++ Example: Moving to a Specific Joint Position

```cpp
void RobotController::FloorRobotSendHome()
{
    // Move floor robot to home joint state
    RCLCPP_INFO_STREAM(get_logger(), "Moving Floor Robot to home position");
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool RobotController::FloorRobotMovetoTarget()
{
    int max_attempts = 3;
    for (int attempt = 1; attempt <= max_attempts; attempt++)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Use more planning time for each successive attempt
        floor_robot_.setPlanningTime(5.0 + (attempt * 5.0));

        bool success = static_cast<bool>(floor_robot_.plan(plan));

        // If plan is found, then execute it
        if (success)
        {
            RCLCPP_INFO(get_logger(), "Plan found, executing...");
            bool execution_success = static_cast<bool>(floor_robot_.execute(plan));
            if (execution_success)
            {
                RCLCPP_INFO(get_logger(), "Plan execution succeeded");
                return true;
            }
            RCLCPP_WARN(get_logger(), "Plan execution failed on attempt %d of %d",
                        attempt, max_attempts);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unable to generate plan on attempt %d of %d",
                        attempt, max_attempts);
        }

        // Small delay before retry
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Try adjusting planning parameters on subsequent attempts
        if (attempt < max_attempts)
        {
            floor_robot_.setMaxVelocityScalingFactor(0.8 - (0.2 * attempt));
            floor_robot_.setMaxAccelerationScalingFactor(0.8 - (0.2 * attempt));

            // Try a different planner
            if (attempt == 2)
            {
                floor_robot_.setPlannerId("RRTConnect");
            }
            else if (attempt == 3)
            {
                floor_robot_.setPlannerId("BiTRRT");
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Failed to move to target after %d attempts", max_attempts);
    return false;
}
```

## Planning Scene Architecture

The planning scene is a fundamental component in MoveIt that represents the robot's environment and current state. It serves as the central data structure for collision checking, motion planning, and visualization.

### Core Components

The planning scene consists of several integrated elements:

#### 1. Robot Model
- Defines the robot's kinematic structure, including links and joints
- Contains information about joint limits and link geometries
- Provides the foundation for all robot-related calculations

#### 2. Robot State
- Represents the current configuration of all robot joints
- Tracks joint positions, velocities, and accelerations
- Updates dynamically as the robot moves
- Forms the starting point for motion planning

#### 3. Collision Objects
- Represent physical obstacles in the environment
- Can be defined using primitive shapes or complex meshes
- Include position and orientation information
- Enable collision-aware motion planning

#### 4. Attached Objects
- Special collision objects that move with the robot
- Represent items being manipulated (e.g., parts held by the gripper)
- Automatically update position as the robot moves
- Allow for collision checking with held objects

### Managing the Planning Scene

#### Python: PlanningSceneMonitor

In Python, the `PlanningSceneMonitor` class provides thread-safe access to the planning scene:

```python
def _add_model_to_planning_scene(
    self, name: str, mesh_file: str, model_pose: Pose, frame_id="world"
) -> bool:
    """
    Add a mesh model to the planning scene using the PlanningSceneMonitor.
    """
    try:
        # Get the full path to the mesh file
        model_path = self._mesh_file_path + mesh_file

        # Create collision object
        collision_object = self._make_mesh(name, model_pose, model_path, frame_id)

        # Add to planning scene using the monitor
        with self._planning_scene_monitor.read_write() as scene:
            # Apply the collision object
            scene.apply_collision_object(collision_object)

            # Update the scene
            scene.current_state.update()

        # Add to our tracking list for later reference
        self._world_collision_objects.append(collision_object)

        return True

    except Exception as e:
        self.get_logger().error(
            f"Error adding model {name} to planning scene: {str(e)}",
        )
        return False
```

#### C++: PlanningSceneInterface

In C++, the planning scene is managed through the `PlanningSceneInterface`:

```cpp
void RobotController::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    planning_scene_.applyCollisionObject(CreateCollisionObject(name, mesh_file, model_pose));
}

moveit_msgs::msg::CollisionObject RobotController::CreateCollisionObject(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("moveit_demo");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    return collision;
}
```

### Adding Environment Models

#### Python Example: Adding Models to Planning Scene

```python
def _add_models_to_planning_scene(self):
    """
    Adds collision models to the MoveIt planning scene
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
    
    return success
```

#### C++ Example: Adding Models to Planning Scene

```cpp
void RobotController::AddModelsToPlanningScene()
{
    RCLCPP_INFO_STREAM(this->get_logger(), BOLD + CHARM_PINK << "Initializing planning scene with collision objects" << RESET);
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    RCLCPP_INFO(get_logger(), "Planning scene initialization complete");
}
```

### Attaching Objects to the Robot

#### Python Example: Attaching Parts to Robot Gripper

```python
def _attach_model_to_floor_gripper(self, part_to_pick: PartMsg, part_pose: Pose):
    """Attach a part to the floor gripper in the planning scene."""
    # Create a part name based on its color and type
    part_name = (
        self._part_colors[part_to_pick.color]
        + "_"
        + self._part_types[part_to_pick.type]
    )

    # Always track the part internally
    self._floor_robot_attached_part = part_to_pick

    # Get the path to the mesh file for the part
    model_path = self._mesh_file_path + self._part_types[part_to_pick.type] + ".stl"

    if not path.exists(model_path):
        self.get_logger().error(f"Mesh file not found: {model_path}")
        return False

    try:
        # Use a single planning scene operation for consistency
        with self._planning_scene_monitor.read_write() as scene:
            # Create the collision object
            co = CollisionObject()
            co.id = part_name
            co.header.frame_id = "world"
            co.header.stamp = self.get_clock().now().to_msg()

            # Create the mesh
            with pyassimp.load(model_path) as assimp_scene:
                if not assimp_scene.meshes:
                    self.get_logger().error(f"No meshes found in {model_path}")
                    return False

                mesh = Mesh()
                # Add triangles
                for face in assimp_scene.meshes[0].faces:
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
                for vertex in assimp_scene.meshes[0].vertices:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    mesh.vertices.append(point)

            # Add the mesh to the collision object
            co.meshes.append(mesh)
            co.mesh_poses.append(part_pose)
            co.operation = CollisionObject.ADD

            # First add to world - this is important!
            scene.apply_collision_object(co)
            
            # Then create the attachment
            aco = AttachedCollisionObject()
            aco.link_name = "floor_gripper"
            aco.object = co
            aco.touch_links = ["floor_gripper", "floor_tool0", "floor_wrist_3_link", 
                            "floor_wrist_2_link", "floor_wrist_1_link", "floor_flange", "floor_ft_frame"]
            
            # Update the state
            scene.current_state.attachBody(part_name, "floor_gripper", aco.touch_links)
            scene.current_state.update()
            
        self.get_logger().info(f"Successfully attached {part_name} to floor gripper")
        return True
            
    except Exception as e:
        self.get_logger().error(f"Error attaching model to gripper: {str(e)}")
        return False
```

#### C++ Example: Attaching Parts to Robot Gripper

```cpp
bool RobotController::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color]
                                                             << " " << part_types_[part_to_pick.type] << " from the bins");

    // Find part and pick it up (code omitted for brevity)
    
    // After picking the part, attach it to the planning scene
    std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
    AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    order_planning_scene_objects_.push_back(part_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.2, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    return true;
}

bool RobotController::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
    // Place part code (omitted for brevity)
    
    // Release and detach part
    FloorRobotSetGripperState(false);

    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type];

    try
    {
        floor_robot_.detachObject(part_name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(get_logger(), "Error detaching object from planning scene: %s", e.what());
        // Continue even if planning scene update fails
    }
    
    return true;
}
```

### Significance in Robotics Applications

A well-maintained planning scene is crucial for:

- **Safe Operation**: Preventing collisions with the environment
- **Efficient Planning**: Providing accurate world information to motion planners
- **Complex Task Execution**: Supporting manipulation of objects
- **Debugging**: Visualizing the robot's understanding of its surroundings
- **Reproducibility**: Ensuring consistent behavior across executions

The planning scene forms the foundation upon which all MoveIt's capabilities are built, making it one of the most important concepts to master when developing robotic applications.

## Key Differences Between Python and C++ Implementations

### Constraints Implementation

- **Python**: Uses explicit `construct_joint_constraint` with `RobotState` and provides constraints to `set_goal_state`
- **C++**: Uses direct methods like `setJointValueTarget` and `setNamedTarget` on the `MoveGroupInterface`

### Planning Scene Management

- **Python**: Uses `PlanningSceneMonitor` with context managers for thread-safe access
- **C++**: Uses `PlanningSceneInterface` with direct method calls
- **Python**: Combines multiple operations in a single atomic update
- **C++**: Performs individual operations with separate method calls

### Error Handling

- **Python**: Uses try-except blocks with detailed error reporting
- **C++**: Uses return values to indicate success/failure
- **Python**: Provides context-based error information
- **C++**: Implements sophisticated retry mechanisms with parameter adjustments

### Attachment Management

- **Python**: Uses `attachBody`/`detachObject` on the planning scene
- **C++**: Uses `attachObject`/`detachObject` on the move group interface


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