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
 * @brief Picks up a kit tray from a kitting tray station and places it on an AGV
 *
 * This function handles the complete workflow for picking a kit tray from either kitting station
 * and placing it on the specified AGV. The process includes:
 * - Locating the tray on either KTS1 or KTS2 stations
 * - Moving the floor robot to the appropriate station
 * - Ensuring the correct gripper is attached
 * - Picking up the tray with adaptive grasping strategies
 * - Transporting the tray to the AGV
 * - Precise placement of the tray on the AGV
 * - Locking the tray to the AGV
 *
 * The function implements a multi-stage approach for both pickup and placement with
 * several error recovery mechanisms to handle potential failures.
 *
 * @param tray_id The ID of the kit tray to pick up
 * @param agv_num The AGV number (1-4) where the tray should be placed
 *
 * @return bool True if the operation was successful, false otherwise
 *
 * @throws None directly, but logs errors when encountering issues
 *
 * @note This function includes multiple safeguards and recovery mechanisms:
 *   - Thread-safe access to shared tray data using mutexes
 *   - Multi-stage approach for reliable tray pickup
 *   - Adaptive retries if station movement fails
 *   - Automated gripper type switching when needed
 *   - Attachment verification using FloorRobotWaitForAttach
 *   - Multi-stage placement with precision control
 *   - Automatic collision avoidance through planning scene updates
 *
 * @warning Requires proper initialization of the robot controller and its subscribers
 *          before execution
 *
 * @see FloorRobotWaitForAttach
 * @see FloorRobotSetGripperState
 * @see FloorRobotChangeGripper
 * @see LockAGVTray
 */
class RobotController : public rclcpp::Node
{
public:
    /// Constructor
    RobotController();

    /// Destructor
    ~RobotController();
    /**
     * @brief Adds collision models to the MoveIt planning scene
     *
     * Populates the planning scene with collision objects for:
     * - Bins (bins 1-8)
     * - Assembly stations (AS1-AS4)
     * - Assembly station briefcases/inserts
     * - Conveyor belt
     * - Kit tray tables (KTS1 and KTS2)
     *
     * These collision objects enable path planning with collision avoidance.
     *
     * @return void
     */
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
    std::mutex conveyor_parts_mutex;
    std::mutex left_bins_mutex_;
    std::mutex right_bins_mutex_;
    std::mutex kts1_trays_mutex_;
    std::mutex kts2_trays_mutex_;

    // Performance metrics and diagnostics
    std::map<std::string, double> task_durations_;
    std::map<std::string, int> success_counts_;
    std::map<std::string, int> failure_counts_;
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

    RobotConfig floor_robot_config_;
    RobotConfig ceiling_robot_config_;

    // Diagnostic publisher
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Service client maps for resource management
    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trigger_clients_;
    std::map<std::string, rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> move_agv_clients_;

    // AGV location
    std::map<int, int> agv_locations_ = {{1, -1}, {2, -1}, {3, -1}, {4, -1}};

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr mutex_cb_group_;
    rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Subscriptions
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_status_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_status_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_status_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_status_sub_;
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_sub_;
    rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_sub_;

    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

    // Assembly States
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

    // Orders List
    ariac_msgs::msg::Order current_order_;
    std::vector<ariac_msgs::msg::Order> orders_;

    unsigned int competition_state_;

    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::Part floor_robot_attached_part_;
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_;
    geometry_msgs::msg::Pose kts2_camera_pose_;
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    geometry_msgs::msg::Pose right_bins_camera_pose_;
    geometry_msgs::msg::Pose conveyor_camera_pose_;
    geometry_msgs::msg::Pose breakbeam_pose_;

    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
    std::vector<std::pair<ariac_msgs::msg::PartPose, rclcpp::Time>> conveyor_parts_;
    std::vector<ariac_msgs::msg::PartPose> conveyor_part_detected_;
    std::vector<ariac_msgs::msg::PartLot> conveyor_parts_expected_;

    // Sensor Callbacks
    bool kts1_camera_recieved_data = false;
    bool kts2_camera_recieved_data = false;
    bool left_bins_camera_recieved_data = false;
    bool right_bins_camera_recieved_data = false;
    bool conveyor_camera_recieved_data = false;
    bool breakbeam_received_data = false;
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

    // ARIAC Services
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

    // Breakbeam parameters
    bool breakbeam_status = false;
    float breakbeam_time_sec;

    // Constants
    double conveyor_speed_ = 0.2;
    double kit_tray_thickness_ = 0.01;
    double drop_height_ = 0.005;
    double pick_offset_ = 0.003;
    double assembly_offset_ = 0.02;
    double battery_grip_offset_ = -0.05;

    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}};

    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };

    // Part heights
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}};

    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };

    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};

    // Joint value targets for kitting stations
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

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

    std::map<std::string, double> floor_conveyor_js_ = {
        {"linear_actuator_joint", 0.0},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -0.9162979},
        {"floor_elbow_joint", 2.04204},
        {"floor_wrist_1_joint", -2.67035},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<int, std::string> agv_destination_ = {
        {ariac_msgs::msg::AGVStatus::KITTING, "kitting"},
        {ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT, "assembly station front"},
        {ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK, "assembly station back"},
        {ariac_msgs::msg::AGVStatus::WAREHOUSE, "warehouse"}};

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