#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unistd.h>

#include <cmath>
#include <chrono>
#include <future>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <ariac_msgs/msg/part_lot.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>

#include <geometry_msgs/msg/pose.hpp>

/**
 * @class RobotController
 * @brief Controller for managing robots in the ARIAC competition environment
 *
 * This class provides a comprehensive robot control system for the ARIAC competition,
 * managing both floor and ceiling robots, planning scenes, gripper operations,
 * and order processing workflows.
 */
class RobotController : public rclcpp::Node
{
public:
    /// Constructor
    RobotController();
    /// Destructor
    ~RobotController();
    void AddModelsToPlanningScene();

    // Floor Robot Public Functions
    void FloorRobotSendHome();
    bool FloorRobotSetGripperState(bool enable);
    bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
    bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);
    bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick);
    bool FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick);
    bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

    // Ceiling Robot Public Functions
    void CeilingRobotSendHome();
    bool CeilingRobotSetGripperState(bool enable);

    // ARIAC Functions
    bool StartCompetition();
    bool EndCompetition();
    bool LockAGVTray(int agv_num);
    bool UnlockAGVTray(int agv_num);
    bool MoveAGV(int agv_num, int destination);
    bool SubmitOrder(std::string order_id);

    bool CompleteOrders();
    bool CompleteKittingTask(ariac_msgs::msg::KittingTask task);
    // Add to the public section of the RobotController class:
    void OnParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

private:
    bool ExecuteTrajectory(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::string &controller_name);
    // Robot Move Functions
    bool FloorRobotMovetoTarget();
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> FloorRobotPlanCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    void FloorRobotWaitForAttach(double timeout);

    bool CeilingRobotMovetoTarget();
    bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> CeilingRobotPlanCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    void CeilingRobotWaitForAttach(double timeout);
    bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
    bool CeilingRobotMoveToAssemblyStation(int station);
    bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
    bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);

    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    // Helper Functions
    void LogPose(geometry_msgs::msg::Pose p);
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    double GetYaw(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);
    moveit_msgs::msg::CollisionObject CreateCollisionObject(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddTemporaryCollisionObjects();
    void RemoveTemporaryCollisionObjects();

    // Service client helper
    template <typename T>
    typename rclcpp::Client<T>::SharedPtr GetOrCreateClient(
        std::map<std::string, typename rclcpp::Client<T>::SharedPtr> &client_map,
        const std::string &srv_name)
    {
        if (client_map.find(srv_name) == client_map.end())
        {
            client_map[srv_name] = this->create_client<T>(srv_name);
        }
        return client_map[srv_name];
    }

    // Thread safety mutexes
    /**
     * @brief Mutex for thread-safe access to conveyor parts data
     *
     * Protects conveyor_parts_ vector during concurrent read/write operations
     * from multiple callbacks and processing threads.
     */
    std::mutex conveyor_parts_mutex;
    /**
     * @brief Mutex for thread-safe access to left bins camera data
     *
     * Protects left_bins_parts_ vector during concurrent read/write operations
     * from sensor callbacks and processing threads.
     */
    std::mutex left_bins_mutex_;
    /**
     * @brief Mutex for thread-safe access to right bins camera data
     *
     * Protects right_bins_parts_ vector during concurrent read/write operations
     * from sensor callbacks and processing threads.
     */
    std::mutex right_bins_mutex_;
    /**
     * @brief Mutex for thread-safe access to KTS1 tray data
     *
     * Protects kts1_trays_ vector during concurrent read/write operations
     * from sensor callbacks and processing threads.
     */
    std::mutex kts1_trays_mutex_;
    /**
     * @brief Mutex for thread-safe access to KTS2 tray data
     *
     * Protects kts2_trays_ vector during concurrent read/write operations
     * from sensor callbacks and processing threads.
     */
    std::mutex kts2_trays_mutex_;

    /**
     * @brief Performance metrics for task durations
     *
     * Maps task names to execution durations for performance analysis
     * and reporting.
     */
    std::map<std::string, double> task_durations_;
    /**
     * @brief Counts of successful task completions
     *
     * Maps task names to success counts for performance tracking
     * and reliability analysis.
     */
    std::map<std::string, int> success_counts_;
    /**
     * @brief Counts of failed task attempts
     *
     * Maps task names to failure counts for identifying problem areas
     * and reliability analysis.
     */
    std::map<std::string, int> failure_counts_;
    /**
     * @brief Task start time for performance tracking
     *
     * Stores the start time of the current task for measuring
     * execution duration.
     */
    rclcpp::Time task_start_time_;

    // Configuration parameters
    struct RobotConfig
    {
        double max_acceleration_scaling_factor;
        double max_velocity_scaling_factor;
        double planning_time;
        int planning_attempts;
        bool allow_replanning;
        int replan_attempts;
    };
    /**
     * @brief Configuration parameters for the floor robot
     *
     * Stores motion planning parameters including velocity scaling,
     * acceleration scaling, planning time, and replanning settings.
     */
    RobotConfig floor_robot_config_;
    /**
     * @brief Configuration parameters for the ceiling robot
     *
     * Stores motion planning parameters including velocity scaling,
     * acceleration scaling, planning time, and replanning settings.
     */
    RobotConfig ceiling_robot_config_;

    /**
     * @brief Publisher for diagnostic information
     *
     * Publishes diagnostic messages about robot controller status
     * and performance metrics.
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    /**
     * @brief Map of trigger service clients
     *
     * Caches service clients for various trigger services to avoid
     * repeated creation of client objects.
     */
    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trigger_clients_;
    /**
     * @brief Map of move AGV service clients
     *
     * Caches service clients for AGV movement services to avoid
     * repeated creation of client objects.
     */
    std::map<std::string, rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> move_agv_clients_;

    /**
     * @brief Map of AGV locations
     *
     * Tracks the current location of each AGV in the competition area.
     * Keys are AGV numbers (1-4), values are location codes.
     */
    std::map<int, int> agv_locations_ = {{1, -1}, {2, -1}, {3, -1}, {4, -1}};

    /**
     * @brief Callback group for service clients
     *
     * Segregates service client callbacks for thread safety and execution control.
     */
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    /**
     * @brief Callback group for mutex-protected operations
     *
     * Ensures mutually exclusive execution of callbacks that access shared resources.
     */
    rclcpp::CallbackGroup::SharedPtr mutex_cb_group_;
    /**
     * @brief Callback group for reentrant callbacks
     *
     * Allows callbacks to be executed concurrently with other callbacks
     * in the same group for improved responsiveness.
     */
    rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;

    /**
     * @brief MoveIt interface for the floor robot
     *
     * Provides motion planning and execution capabilities for the floor robot.
     */
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    /**
     * @brief MoveIt interface for the ceiling robot
     *
     * Provides motion planning and execution capabilities for the ceiling robot.
     */
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;
    /**
     * @brief MoveIt planning scene interface
     *
     * Provides access to the planning scene for collision object management.
     */
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
    /**
     * @brief Time-optimal trajectory generator
     *
     * Optimizes trajectory timing for smooth and efficient robot motion.
     */
    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    /**
     * @brief TF2 buffer for transform lookups
     *
     * Stores and manages coordinate transformations between different frames.
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    /**
     * @brief TF2 transform listener
     *
     * Listens for transformations published to the /tf topic and stores them in the buffer.
     */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    /**
     * @brief Subscription for order messages
     *
     * Receives order information from the competition system.
     */
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;

    /**
     * @brief Subscription for AGV1 status messages
     *
     * Receives status updates for AGV1.
     */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_status_sub_;
    /**
     * @brief Subscription for AGV2 status messages
     *
     * Receives status updates for AGV2.
     */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_status_sub_;
    /**
     * @brief Subscription for AGV3 status messages
     *
     * Receives status updates for AGV3.
     */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_status_sub_;
    /**
     * @brief Subscription for AGV4 status messages
     *
     * Receives status updates for AGV4.
     */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_status_sub_;
    /**
     * @brief Subscription for competition state messages
     *
     * Receives updates about the current state of the competition.
     */
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
    /**
     * @brief Subscription for KTS1 camera images
     *
     * Receives images from the camera above kitting tray station 1.
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
    /**
     * @brief Subscription for KTS2 camera images
     *
     * Receives images from the camera above kitting tray station 2.
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
    /**
     * @brief Subscription for left bins camera images
     *
     * Receives images from the camera above the left parts bins.
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    /**
     * @brief Subscription for right bins camera images
     *
     * Receives images from the camera above the right parts bins.
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    /**
     * @brief Subscription for conveyor camera images
     *
     * Receives images from the camera above the conveyor belt.
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_;
    /**
     * @brief Subscription for breakbeam sensor status
     *
     * Receives breakbeam sensor events from the conveyor belt sensor.
     */
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_sub_;
    /**
     * @brief Subscription for conveyor parts notifications
     *
     * Receives information about parts expected on the conveyor belt.
     */
    rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_sub_;
    /**
     * @brief Subscription for floor robot gripper state
     *
     * Receives status updates about the floor robot gripper.
     */
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    /**
     * @brief Subscription for ceiling robot gripper state
     *
     * Receives status updates about the ceiling robot gripper.
     */
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;

    /**
     * @brief Subscription for assembly station 1 state
     *
     * Receives state updates from assembly station 1.
     */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    /**
     * @brief Subscription for assembly station 2 state
     *
     * Receives state updates from assembly station 2.
     */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    /**
     * @brief Subscription for assembly station 3 state
     *
     * Receives state updates from assembly station 3.
     */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    /**
     * @brief Subscription for assembly station 4 state
     *
     * Receives state updates from assembly station 4.
     */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;
    /**
     * @brief Map of assembly station states
     *
     * Tracks the current state of each assembly station.
     * Keys are station numbers (1-4), values are assembly states.
     */
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;
    /**
     * @brief Currently active order
     *
     * Stores the order currently being processed.
     */
    ariac_msgs::msg::Order current_order_;

    /**
     * @brief Queue of pending orders
     *
     * Stores orders that have been received but not yet processed.
     */
    std::vector<ariac_msgs::msg::Order> orders_;
    /**
     * @brief Current competition state
     *
     * Stores the current state of the competition (IDLE, READY, STARTED, etc.).
     */
    unsigned int competition_state_;

    /**
     * @brief Floor robot gripper state
     *
     * Tracks the current state of the floor robot's gripper.
     */
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    /**
     * @brief Part currently attached to floor robot
     *
     * Stores information about the part currently held by the floor robot.
     */
    ariac_msgs::msg::Part floor_robot_attached_part_;
    /**
     * @brief Ceiling robot gripper state
     *
     * Tracks the current state of the ceiling robot's gripper.
     */
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    /**
     * @brief Part currently attached to ceiling robot
     *
     * Stores information about the part currently held by the ceiling robot.
     */
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    /**
     * @brief Pose of the KTS1 camera in world frame
     *
     * Stores the position and orientation of the camera at kitting tray station 1.
     */
    geometry_msgs::msg::Pose kts1_camera_pose_;
    /**
     * @brief Pose of the KTS2 camera in world frame
     *
     * Stores the position and orientation of the camera at kitting tray station 2.
     */
    geometry_msgs::msg::Pose kts2_camera_pose_;

    /**
     * @brief Pose of the left bins camera in world frame
     *
     * Stores the position and orientation of the camera above the left parts bins.
     */
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    /**
     * @brief Pose of the right bins camera in world frame
     *
     * Stores the position and orientation of the camera above the right parts bins.
     */
    geometry_msgs::msg::Pose right_bins_camera_pose_;
    /**
     * @brief Pose of the conveyor camera in world frame
     *
     * Stores the position and orientation of the camera above the conveyor belt.
     */
    geometry_msgs::msg::Pose conveyor_camera_pose_;

    /**
     * @brief Pose of the breakbeam sensor in world frame
     *
     * Stores the position and orientation of the breakbeam sensor on the conveyor.
     */
    geometry_msgs::msg::Pose breakbeam_pose_;

    /**
     * @brief Vector of tray poses at KTS1
     *
     * Stores information about trays detected at kitting tray station 1.
     */
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    /**
     * @brief Vector of tray poses at KTS2
     *
     * Stores information about trays detected at kitting tray station 2.
     */
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;
    /**
     * @brief Vector of part poses in left bins
     *
     * Stores information about parts detected in the left bins.
     */
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    /**
     * @brief Vector of part poses in right bins
     *
     * Stores information about parts detected in the right bins.
     */
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
    /**
     * @brief Vector of parts detected on the conveyor with timestamps
     *
     * Stores information about parts detected on the conveyor belt and
     * the time they were detected, for tracking and interception.
     */
    std::vector<std::pair<ariac_msgs::msg::PartPose, rclcpp::Time>> conveyor_parts_;
    /**
     * @brief Vector of part poses detected by the conveyor camera
     *
     * Stores information about parts currently visible to the conveyor camera.
     */
    std::vector<ariac_msgs::msg::PartPose> conveyor_part_detected_;
    /**
     * @brief Vector of parts expected on the conveyor
     *
     * Stores information about parts that will be coming down the conveyor
     * based on competition announcements.
     */
    std::vector<ariac_msgs::msg::PartLot> conveyor_parts_expected_;

    /**
     * @brief Flag indicating if data has been received from KTS1 camera
     *
     * Used to track when the first data from this sensor is received.
     */
    bool kts1_camera_recieved_data = false;
    /**
     * @brief Flag indicating if data has been received from KTS2 camera
     *
     * Used to track when the first data from this sensor is received.
     */
    bool kts2_camera_recieved_data = false;
    /**
     * @brief Flag indicating if data has been received from left bins camera
     *
     * Used to track when the first data from this sensor is received.
     */
    bool left_bins_camera_recieved_data = false;
    /**
     * @brief Flag indicating if data has been received from right bins camera
     *
     * Used to track when the first data from this sensor is received.
     */
    bool right_bins_camera_recieved_data = false;
    /**
     * @brief Flag indicating if data has been received from conveyor camera
     *
     * Used to track when the first data from this sensor is received.
     */
    bool conveyor_camera_recieved_data = false;
    /**
     * @brief Flag indicating if data has been received from breakbeam sensor
     *
     * Used to track when the first data from this sensor is received.
     */
    bool breakbeam_received_data = false;
    /**
     * @brief Flag indicating if data has been received about conveyor parts
     *
     * Used to track when the first data about expected conveyor parts is received.
     */
    bool conveyor_parts_recieved_data = false;

    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void breakbeam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    void conveyor_parts_cb(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);

    // Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    // AGV Status Callback
    void agv1_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv2_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv3_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv4_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);

    // Orders Callback
    void TopicOrdersCallback(const ariac_msgs::msg::Order::ConstSharedPtr msg);

    // Competition state callback
    void TopicCompetitionStateCallback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

    /**
     * @brief Service client for quality checking
     *
     * Used to request quality checks for completed kits.
     */
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    /**
     * @brief Service client for getting pre-assembly poses
     *
     * Used to request pose information for parts before assembly.
     */
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
    /**
     * @brief Service client for changing floor robot tools
     *
     * Used to change between different gripper types on the floor robot.
     */
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    /**
     * @brief Service client for controlling the floor robot gripper
     *
     * Used to enable or disable the vacuum gripper on the floor robot.
     */
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
    /**
     * @brief Service client for controlling the ceiling robot gripper
     *
     * Used to enable or disable the vacuum gripper on the ceiling robot.
     */
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

    /**
     * @brief Current state of the breakbeam sensor
     *
     * Indicates whether the breakbeam is currently detecting an object.
     */
    bool breakbeam_status = false;
    /**
     * @brief Timestamp of the last breakbeam event
     *
     * Stores the time when the breakbeam was last triggered, used for
     * timing calculations in conveyor tracking.
     */
    float breakbeam_time_sec;

    /**
     * @brief Speed of the conveyor belt in m/s
     *
     * Used for calculating part positions and planning interception timing.
     */
    double conveyor_speed_ = 0.2;
    /**
     * @brief Thickness of the kit tray in meters
     *
     * Used for calculating precise placement positions on trays.
     */
    double kit_tray_thickness_ = 0.01;

    /**
     * @brief Height offset for part dropping in meters
     *
     * Small offset to prevent collision when placing parts.
     */
    double drop_height_ = 0.005;
    /**
     * @brief Height offset for part picking in meters
     *
     * Small offset to ensure proper vacuum gripper contact during pickup.
     */
    double pick_offset_ = 0.003;
    /**
     * @brief Height offset for assembly operations in meters
     *
     * Distance offset used during assembly operations.
     */
    double assembly_offset_ = 0.02;
    /**
     * @brief Grip offset for battery parts in meters
     *
     * Special offset needed for properly grasping battery parts.
     */
    double battery_grip_offset_ = -0.05;
    /**
     * @brief Map of part type codes to human-readable names
     *
     * Used for logging and diagnostics to translate numeric type codes.
     */
    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}};
    /**
     * @brief Map of part color codes to human-readable names
     *
     * Used for logging and diagnostics to translate numeric color codes.
     */
    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };
    /**
     * @brief Map of part types to their heights in meters
     *
     * Used for calculating precise pick and place positions.
     */
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}};
    /**
     * @brief Map of quadrant numbers to position offsets
     *
     * Defines the x,y offsets for each quadrant on the kit tray.
     */
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };
    /**
     * @brief Map of named positions to rail positions in meters
     *
     * Defines the linear actuator joint positions for reaching key locations.
     */
    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};

    /**
     * @brief Joint values for floor robot at KTS1
     *
     * Predefined joint positions for the floor robot at kitting tray station 1.
     */
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};
    /**
     * @brief Joint values for floor robot at KTS2
     *
     * Predefined joint positions for the floor robot at kitting tray station 2.
     */
    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};
    /**
     * @brief Joint values for ceiling robot at assembly station 1
     *
     * Predefined joint positions for the ceiling robot at assembly station 1.
     */
    std::map<std::string, double> ceiling_as1_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}};

    /**
     * @brief Joint values for ceiling robot at assembly station 2
     *
     * Predefined joint positions for the ceiling robot at assembly station 2.
     */
    std::map<std::string, double> ceiling_as2_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}};
    /**
     * @brief Joint values for ceiling robot at assembly station 3
     *
     * Predefined joint positions for the ceiling robot at assembly station 3.
     */
    std::map<std::string, double> ceiling_as3_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}};
    /**
     * @brief Joint values for ceiling robot at assembly station 4
     *
     * Predefined joint positions for the ceiling robot at assembly station 4.
     */
    std::map<std::string, double> ceiling_as4_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}};
    /**
     * @brief Joint values for floor robot at conveyor
     *
     * Predefined joint positions for the floor robot at the conveyor belt.
     */
    std::map<std::string, double> floor_conveyor_js_ = {
        {"linear_actuator_joint", 0.0},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -0.9162979},
        {"floor_elbow_joint", 2.04204},
        {"floor_wrist_1_joint", -2.67035},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};
    /**
     * @brief Map of AGV destination codes to human-readable names
     *
     * Used for logging and diagnostics to translate destination codes.
     */
    std::map<int, std::string> agv_destination_ = {
        {ariac_msgs::msg::AGVStatus::KITTING, "kitting"},
        {ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT, "assembly station front"},
        {ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK, "assembly station back"},
        {ariac_msgs::msg::AGVStatus::WAREHOUSE, "warehouse"}};
    /**
     * @brief List of planning scene objects added for the current order
     *
     * Tracks collision objects that should be cleaned up when an order completes.
     */
    std::vector<std::string> order_planning_scene_objects_;

    // Basic color definitions
    const std::string BLACK = "\033[30m";
    const std::string RED = "\033[31m";
    const std::string GREEN = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string BLUE = "\033[34m";
    const std::string MAGENTA = "\033[35m";
    const std::string CYAN = "\033[36m";
    const std::string WHITE = "\033[37m";
    const std::string RESET = "\033[0m";

    // Bright versions
    const std::string BRIGHT_BLACK = "\033[90m"; // Usually appears as dark gray
    const std::string BRIGHT_RED = "\033[91m";
    const std::string BRIGHT_GREEN = "\033[92m";
    const std::string BRIGHT_YELLOW = "\033[93m";
    const std::string BRIGHT_BLUE = "\033[94m";
    const std::string BRIGHT_MAGENTA = "\033[95m";
    const std::string BRIGHT_CYAN = "\033[96m";
    const std::string BRIGHT_WHITE = "\033[97m";

    // Text styles
    const std::string BOLD = "\033[1m";
    const std::string UNDERLINE = "\033[4m";
    const std::string ITALIC = "\033[3m"; // Not supported by all terminals

    // Function to create a custom color from RGB values (true color)
    std::string rgb_color(int r, int g, int b)
    {
        return "\033[38;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
    }

    // Function to create a custom color from 256-color palette
    std::string color_256(int code)
    {
        return "\033[38;5;" + std::to_string(code) + "m";
    }

    // Example usage
    const std::string ORANGE = rgb_color(255, 165, 0);
    const std::string PURPLE = rgb_color(128, 0, 128);
    const std::string LIGHT_BLUE = rgb_color(173, 216, 230);

    const std::string CHARM_PINK = rgb_color(235, 143, 166);

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    std::string current_operation_mode_;
    bool StartOperation();
    bool ExecutePickPlacePartOperation();
    bool ExecutePickPlaceTrayOperation();
    rclcpp::TimerBase::SharedPtr competition_timer_;
    bool competition_started_ = false;
    void CompetitionTimerCallback();
    bool operation_started_{false};
};