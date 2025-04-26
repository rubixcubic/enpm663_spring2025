#include <moveit_demo/minimal_demo.hpp>
#include <random>

RobotController::RobotController()
    : Node("moveit_demo"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
    // In the RobotController constructor:
    this->declare_parameter("operation_mode", "pick_place_tray");
    current_operation_mode_ = this->get_parameter("operation_mode").as_string();

    // Subscribe to parameter events
    parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&RobotController::OnParameterEvent, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Robot controller initialized with operation mode: %s",
                current_operation_mode_.c_str());

    // Use upper joint velocity and acceleration limits
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);
    floor_robot_.setPlanningTime(10.0);
    floor_robot_.setNumPlanningAttempts(5);
    floor_robot_.allowReplanning(true);
    floor_robot_.setReplanAttempts(5);

    ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
    ceiling_robot_.setMaxVelocityScalingFactor(1.0);
    ceiling_robot_.setPlanningTime(10.0);
    ceiling_robot_.setNumPlanningAttempts(5);
    ceiling_robot_.allowReplanning(true);
    ceiling_robot_.setReplanAttempts(5);
    // Subscribe to topics
    rclcpp::SubscriptionOptions mutex_options;
    rclcpp::SubscriptionOptions reentrant_options;

    mutex_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reentrant_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mutex_options.callback_group = mutex_cb_group_;
    reentrant_options.callback_group = reentrant_cb_group_;

    orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1,
                                                                    std::bind(&RobotController::TopicOrdersCallback, this, std::placeholders::_1), mutex_options);

    competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1, std::bind(&RobotController::TopicCompetitionStateCallback, this, std::placeholders::_1), mutex_options);

    kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::kts1_camera_cb, this, std::placeholders::_1), reentrant_options);

    kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::kts2_camera_cb, this, std::placeholders::_1), reentrant_options);

    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::left_bins_camera_cb, this, std::placeholders::_1), reentrant_options);

    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::right_bins_camera_cb, this, std::placeholders::_1), reentrant_options);

    conveyor_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/conveyor_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::conveyor_camera_cb, this, std::placeholders::_1), reentrant_options);

    breakbeam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/conveyor_breakbeam/change", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::breakbeam_cb, this, std::placeholders::_1), reentrant_options);

    conveyor_parts_sub_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>(
        "/ariac/conveyor_parts", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::conveyor_parts_cb, this, std::placeholders::_1), mutex_options);

    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::floor_gripper_state_cb, this, std::placeholders::_1), reentrant_options);

    ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&RobotController::ceiling_gripper_state_cb, this, std::placeholders::_1), reentrant_options);

    agv1_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
        "/ariac/agv1_status", 10,
        std::bind(&RobotController::agv1_status_cb, this, std::placeholders::_1), mutex_options);

    agv2_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
        "/ariac/agv2_status", 10,
        std::bind(&RobotController::agv2_status_cb, this, std::placeholders::_1), mutex_options);

    agv3_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
        "/ariac/agv3_status", 10,
        std::bind(&RobotController::agv3_status_cb, this, std::placeholders::_1), mutex_options);

    agv4_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
        "/ariac/agv4_status", 10,
        std::bind(&RobotController::agv4_status_cb, this, std::placeholders::_1), mutex_options);

    // Initialize service clients
    floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");

    competition_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotController::CompetitionTimerCallback, this));

    RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Initialization successful. " << RESET);
}

RobotController::~RobotController()
{
    floor_robot_.~MoveGroupInterface();
    ceiling_robot_.~MoveGroupInterface();
}
void RobotController::CompetitionTimerCallback()
{

    // If competition is in READY state and hasn't been started yet
    if (competition_state_ == ariac_msgs::msg::CompetitionState::READY && !competition_started_)
    {

        if (StartCompetition())
        {
            RCLCPP_INFO(get_logger(), "Competition started successfully");
            competition_started_ = true;

        }
    }
}

void RobotController::OnParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    // Check if this is our node's parameter
    if (event->node == this->get_fully_qualified_name())
    {
        for (auto &changed_param : event->changed_parameters)
        {
            if (changed_param.name == "operation_mode")
            {
                std::string new_mode = changed_param.value.string_value;

                if (new_mode != current_operation_mode_)
                {
                    RCLCPP_INFO(get_logger(), "Operation mode changed from %s to %s",
                                current_operation_mode_.c_str(), new_mode.c_str());
                    current_operation_mode_ = new_mode;

                    // Start the operation if competition is already running
                    if (competition_state_ == ariac_msgs::msg::CompetitionState::STARTED)
                    {
                        StartOperation();
                    }
                }
            }
        }
    }
}

bool RobotController::StartOperation()
{
    RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Starting operation: " << current_operation_mode_ << RESET);

    AddModelsToPlanningScene();


    if (current_operation_mode_ == "pick_place_tray")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Executing tray pick and place operation" << RESET);

        return ExecutePickPlaceTrayOperation();
    }
    else if (current_operation_mode_ == "pick_place_part")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Executing part pick and place operation" << RESET);
        return ExecutePickPlacePartOperation();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown operation mode: %s", current_operation_mode_.c_str());
        return false;
    }
    EndCompetition();
}

void RobotController::TopicOrdersCallback(
    const ariac_msgs::msg::Order::ConstSharedPtr msg)
{
    orders_.push_back(*msg);
}

bool RobotController::ExecutePickPlaceTrayOperation()
{
    // Wait for first order to be published
    while (orders_.size() == 0)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for orders...");
        // Check if we should exit
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            return false;
        }
        // Small sleep to avoid busy waiting
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_logger(), "Starting tray pick and place operation");

    // Get the first order
    current_order_ = orders_.front();
    orders_.erase(orders_.begin());

    // Only proceed if it's a kitting task
    if (current_order_.type != ariac_msgs::msg::Order::KITTING)
    {
        RCLCPP_ERROR(get_logger(), "Order is not a kitting task");
        return false;
    }

    // Move floor robot to safe starting position
    FloorRobotSendHome();

    // Ensure AGV is at kitting station
    if (agv_locations_[current_order_.kitting_task.agv_number] != ariac_msgs::msg::AGVStatus::KITTING)
    {
        RCLCPP_INFO(get_logger(), "Moving AGV %d to kitting station", current_order_.kitting_task.agv_number);
        if (!MoveAGV(current_order_.kitting_task.agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING))
        {
            RCLCPP_ERROR(get_logger(), "Failed to move AGV to kitting station");
            return false;
        }
    }

    // Pick and place the tray only
    bool success = FloorRobotPickandPlaceTray(current_order_.kitting_task.tray_id,
                                              current_order_.kitting_task.agv_number);

    if (success)
    {
        RCLCPP_INFO(get_logger(), "Successfully picked and placed tray");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to pick and place tray");
    }

    // Return to home position
    FloorRobotSendHome();

    return success;
}

bool RobotController::ExecutePickPlacePartOperation()
{
    // Wait for first order to be published
    while (orders_.size() == 0)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for orders...");
        // Check if we should exit
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            return false;
        }
        // Small sleep to avoid busy waiting
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_logger(), "Starting part pick and place operation");

    // Get the first order
    current_order_ = orders_.front();
    orders_.erase(orders_.begin());

    // Only proceed if it's a kitting task
    if (current_order_.type != ariac_msgs::msg::Order::KITTING)
    {
        RCLCPP_ERROR(get_logger(), "Order is not a kitting task");
        return false;
    }

    // Move floor robot to safe starting position
    FloorRobotSendHome();

    // Ensure AGV is at kitting station and has a tray
    int agv_num = current_order_.kitting_task.agv_number;

    if (agv_locations_[agv_num] != ariac_msgs::msg::AGVStatus::KITTING)
    {
        RCLCPP_INFO(get_logger(), "Moving AGV %d to kitting station", agv_num);
        if (!MoveAGV(agv_num, ariac_msgs::srv::MoveAGV::Request::KITTING))
        {
            RCLCPP_ERROR(get_logger(), "Failed to move AGV to kitting station");
            return false;
        }
    }

    // Select a random part to pick up
    ariac_msgs::msg::Part part_to_pick{};
    bool found_part{false};

    // Check left bins first
    {
        std::lock_guard<std::mutex> lock(left_bins_mutex_);
        if (!left_bins_parts_.empty())
        {
            // Pick a random part from left bins
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dist(0, left_bins_parts_.size() - 1);
            int idx = dist(gen);
            part_to_pick = left_bins_parts_[idx].part;
            found_part = true;
        }
    }

    // If no part in left bins, check right bins
    if (!found_part)
    {
        std::lock_guard<std::mutex> lock(right_bins_mutex_);
        if (!right_bins_parts_.empty())
        {
            // Pick a random part from right bins
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dist(0, right_bins_parts_.size() - 1);
            int idx = dist(gen);
            part_to_pick = right_bins_parts_[idx].part;
            found_part = true;
        }
    }

    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "No parts found in bins");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Selected a %s %s part to pick",
                part_colors_[part_to_pick.color].c_str(),
                part_types_[part_to_pick.type].c_str());

    // Pick the part
    if (!FloorRobotPickBinPart(part_to_pick))
    {
        RCLCPP_ERROR(get_logger(), "Failed to pick part from bins");
        return false;
    }

    // Place on a random quadrant
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> quadrant_dist(1, 4);
    int quadrant = quadrant_dist(gen);

    RCLCPP_INFO(get_logger(), "Placing part in quadrant %d", quadrant);

    bool success = FloorRobotPlacePartOnKitTray(agv_num, quadrant);

    if (success)
    {
        RCLCPP_INFO(get_logger(), "Successfully placed part on tray");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to place part on tray");
    }

    // Return to home position
    FloorRobotSendHome();

    return success;
}

void RobotController::TopicCompetitionStateCallback(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;

    // RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Competition state: " << competition_state_ << RESET);

    // If competition just started, start the operation
    if (operation_started_ == true)
        return;

    if (competition_state_ == ariac_msgs::msg::CompetitionState::STARTED or competition_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
    {
        StartOperation();
        operation_started_ = true;
    }
}

void RobotController::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts1_camera_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from kts1 camera");
        kts1_camera_recieved_data = true;
    }

    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void RobotController::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts2_camera_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from kts2 camera");
        kts2_camera_recieved_data = true;
    }

    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void RobotController::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!left_bins_camera_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from left bins camera");
        left_bins_camera_recieved_data = true;
    }

    left_bins_parts_ = msg->part_poses;
    left_bins_camera_pose_ = msg->sensor_pose;
}

void RobotController::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!right_bins_camera_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from right bins camera");
        right_bins_camera_recieved_data = true;
    }

    right_bins_parts_ = msg->part_poses;
    right_bins_camera_pose_ = msg->sensor_pose;
}

void RobotController::conveyor_parts_cb(
    const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg)
{
    if (!conveyor_parts_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from conveyor parts");
        conveyor_parts_recieved_data = true;
    }

    conveyor_parts_expected_ = msg->parts;
}

void RobotController::conveyor_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!conveyor_camera_recieved_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from conveyor camera");
        conveyor_camera_recieved_data = true;
    }

    conveyor_part_detected_ = msg->part_poses;
    conveyor_camera_pose_ = msg->sensor_pose;
}

void RobotController::breakbeam_cb(
    const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg)
{
    if (!breakbeam_received_data)
    {
        RCLCPP_DEBUG(get_logger(), "Received data from conveyor breakbeam");
        breakbeam_received_data = true;
        breakbeam_pose_ = FrameWorldPose(msg->header.frame_id);
    }

    breakbeam_status = msg->object_detected;
    ariac_msgs::msg::PartPose part_to_add;
    auto detection_time = now();
    float prev_distance = 0;
    int count = 0;

    if (breakbeam_status)
    {
        // Lock conveyor_parts_ mutex
        std::lock_guard<std::mutex> lock(conveyor_parts_mutex);

        for (auto part : conveyor_part_detected_)
        {
            geometry_msgs::msg::Pose part_pose = MultiplyPose(conveyor_camera_pose_, part.pose);
            float distance = abs(part_pose.position.y - breakbeam_pose_.position.y);
            if (count == 0)
            {
                part_to_add = part;
                detection_time = now();
                prev_distance = distance;
                count++;
            }
            else
            {
                if (distance < prev_distance)
                {
                    part_to_add = part;
                    detection_time = now();
                    prev_distance = distance;
                }
            }
        }
        conveyor_parts_.emplace_back(part_to_add, detection_time);
    }
}

void RobotController::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

void RobotController::ceiling_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    ceiling_gripper_state_ = *msg;
}

void RobotController::agv1_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
    agv_locations_[1] = msg->location;
}

void RobotController::agv2_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
    agv_locations_[2] = msg->location;
}

void RobotController::agv3_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
    agv_locations_[3] = msg->location;
}

void RobotController::agv4_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
    agv_locations_[4] = msg->location;
}

geometry_msgs::msg::Pose RobotController::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void RobotController::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose RobotController::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose RobotController::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double RobotController::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion RobotController::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
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

void RobotController::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{

    planning_scene_.applyCollisionObject(CreateCollisionObject(name, mesh_file, model_pose));
}

/**
 * @brief Adds all required collision models to the MoveIt planning scene
 *
 * Initializes the planning scene with collision objects representing:
 * - Storage bins (bins 1-8)
 * - Assembly stations (AS1-AS4)
 * - Assembly station inserts/briefcases
 * - Conveyor belt
 * - Kit tray tables (KTS1 and KTS2)
 *
 * This comprehensive scene enables collision-aware path planning for both robots.
 *
 * @return void
 *
 * @note If any object cannot be added, the function will log a warning but continue
 *       with the remaining objects
 */
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

    // Add assembly briefcases
    std::map<std::string, std::string> assembly_inserts = {
        {"as1_insert", "as1_insert_frame"},
        {"as2_insert", "as2_insert_frame"},
        {"as3_insert", "as3_insert_frame"},
        {"as4_insert", "as4_insert_frame"}};

    for (auto const &insert : assembly_inserts)
    {
        try
        {
            geometry_msgs::msg::Pose insert_pose = FrameWorldPose(insert.second);
            AddModelToPlanningScene(insert.first, "assembly_insert.stl", insert_pose);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Failed to add assembly insert %s: %s",
                        insert.first.c_str(), e.what());
        }
    }

    // Add conveyor
    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    // Add kit tray tables
    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);

    RCLCPP_INFO(get_logger(), "Planning scene initialization complete");
}

geometry_msgs::msg::Quaternion RobotController::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
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

bool RobotController::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
    // Add incremental path planning with segment subdivision
    double path_fraction = 0.0;
    int max_attempts = 5;
    double eef_step = 0.01; // 1cm steps

    for (int attempt = 1; attempt <= max_attempts; attempt++)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;

        // Try with current waypoints
        path_fraction = floor_robot_.computeCartesianPath(
            waypoints, eef_step, 0.0, trajectory, avoid_collisions);

        if (path_fraction >= 0.9)
        {
            // Success! Now optimize and execute
            robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
            rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
            totg_.computeTimeStamps(rt, vsf, asf);
            rt.getRobotTrajectoryMsg(trajectory);

            RCLCPP_INFO(get_logger(), "Successfully planned Cartesian path with %.2f%% coverage",
                        path_fraction * 100.0);

            return static_cast<bool>(floor_robot_.execute(trajectory));
        }

        // Failed - try with increased step size or segmentation
        RCLCPP_WARN(get_logger(), "Cartesian planning attempt %d achieved only %.2f%% coverage",
                    attempt, path_fraction * 100.0);

        if (attempt < max_attempts)
        {
            // First try with a larger step size
            eef_step += 0.01 * attempt;

            // If multiple waypoints, try planning segment by segment
            if (waypoints.size() > 1 && attempt > 2)
            {
                RCLCPP_INFO(get_logger(), "Trying segment by segment planning");
                bool overall_success = true;
                geometry_msgs::msg::Pose start_pose = floor_robot_.getCurrentPose().pose;

                for (size_t i = 0; i < waypoints.size(); i++)
                {
                    std::vector<geometry_msgs::msg::Pose> segment = {start_pose, waypoints[i]};

                    // Try planning this segment
                    moveit_msgs::msg::RobotTrajectory segment_traj;
                    double segment_fraction = floor_robot_.computeCartesianPath(
                        segment, eef_step, 0.0, segment_traj, avoid_collisions);

                    if (segment_fraction >= 0.9)
                    {
                        // Optimize and execute this segment
                        robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
                        rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), segment_traj);
                        totg_.computeTimeStamps(rt, vsf, asf);
                        rt.getRobotTrajectoryMsg(segment_traj);

                        RCLCPP_INFO(get_logger(), "Executing segment %zu to waypoint %zu", i + 1, i + 1);
                        if (!floor_robot_.execute(segment_traj))
                        {
                            RCLCPP_ERROR(get_logger(), "Failed to execute segment %zu", i + 1);
                            overall_success = false;
                            break;
                        }

                        // Update start pose for next segment
                        start_pose = waypoints[i];
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to plan segment %zu with coverage %.2f%%",
                                     i + 1, segment_fraction * 100.0);
                        overall_success = false;
                        break;
                    }
                }

                if (overall_success)
                {
                    RCLCPP_INFO(get_logger(), "Successfully executed all segments");
                    return true;
                }

                // If we reach here, at least one segment failed
                // Try a different approach in the next iteration
            }

            // If a single waypoint or early attempts, try with joint space planning
            if (waypoints.size() == 1 && attempt == max_attempts - 1)
            {
                RCLCPP_INFO(get_logger(), "Trying joint-space planning for single waypoint");
                floor_robot_.setPoseTarget(waypoints[0]);
                return FloorRobotMovetoTarget();
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Failed to plan Cartesian path after %d attempts", max_attempts);
    return false;
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> RobotController::FloorRobotPlanCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return std::make_pair(false, trajectory);
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return std::make_pair(true, trajectory);
}

void RobotController::FloorRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.01, 0.01, true);

        usleep(500);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

// void RobotController::FloorRobotWaitForAttach(double timeout)
// {
//     // Wait for part to be attached with more efficient strategy
//     rclcpp::Time start = now();
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

//     // First try waiting a short time without moving
//     rclcpp::sleep_for(std::chrono::milliseconds(200));
//     if (floor_gripper_state_.attached)
//     {
//         RCLCPP_INFO(get_logger(), "Part attached immediately");
//         return;
//     }

//     int micro_adjustment_count = 0;
//     const int max_micro_adjustments = 5;
//     double adjustment_size = 0.001; // 1mm initial adjustment

//     while (!floor_gripper_state_.attached)
//     {
//         waypoints.clear();
//         waypoints.push_back(starting_pose);
//         FloorRobotMoveCartesian(waypoints, 0.01, 0.01, false);

//         // Give a short time for attachment to register
//         rclcpp::sleep_for(std::chrono::milliseconds(200));

//         // Increase adjustment size for next attempt
//         adjustment_size += 0.0005; // Gradually increase to 0.0015, 0.002, etc.

//         if (now() - start > rclcpp::Duration::from_seconds(timeout))
//         {
//             RCLCPP_ERROR(get_logger(), "Unable to pick up object: timeout");
//             return;
//         }
//     }

//     if (floor_gripper_state_.attached)
//     {
//         RCLCPP_INFO(get_logger(), "Successfully attached after %d micro-adjustments",
//                     micro_adjustment_count);
//     }
//     else
//     {
//         RCLCPP_ERROR(get_logger(), "Failed to attach after %d micro-adjustments",
//                      micro_adjustment_count);
//     }
// }

void RobotController::FloorRobotSendHome()
{
    // Move floor robot to home joint state
    RCLCPP_INFO_STREAM(get_logger(), "Moving Floor Robot to home position");
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool RobotController::FloorRobotSetGripperState(bool enable)
{
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto future = floor_robot_gripper_enable_->async_send_request(request);
    future.wait();

    if (!future.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

bool RobotController::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
    // Move gripper into tool changer
    RCLCPP_INFO_STREAM(get_logger(), "Changing gripper to " << gripper_type << " gripper");

    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true))
        return false;

    // Call service to change gripper
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    if (gripper_type == "trays")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    }
    else if (gripper_type == "parts")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    auto future = floor_robot_tool_changer_->async_send_request(request);
    future.wait();
    if (!future.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
        return false;
    }

    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1, true))
        return false;

    return true;
}
/////////////////////////////
bool RobotController::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
    RCLCPP_INFO_STREAM(get_logger(), BOLD + ORANGE << "Attempting to pick up kit tray " << tray_id << " and place on AGV " << agv_num << RESET);

    // Check if kit tray is on one of the two tables with better error handling
    geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;

    // Check table 1 with mutex protection
    {
        std::lock_guard<std::mutex> lock(kts1_trays_mutex_);
        for (auto tray : kts1_trays_)
        {
            if (tray.id == tray_id)
            {
                station = "kts1";
                tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
                found_tray = true;
                break;
            }
        }
    }

    // Check table 2 with mutex protection
    if (!found_tray)
    {
        std::lock_guard<std::mutex> lock(kts2_trays_mutex_);
        for (auto tray : kts2_trays_)
        {
            if (tray.id == tray_id)
            {
                station = "kts2";
                tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
                found_tray = true;
                break;
            }
        }
    }

    if (!found_tray)
    {
        RCLCPP_ERROR(get_logger(), "Could not find kit tray %d on either station", tray_id);
        return false;
    }

    double tray_rotation = GetYaw(tray_pose);

    RCLCPP_INFO_STREAM(get_logger(), BOLD + ORANGE << "Found kit tray " << tray_id << " on kitting tray station " << station << " moving to pick location" << RESET);

    // Move floor robot to the corresponding kit tray table - Use joint space motion for reliability
    if (station == "kts1")
    {
        floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        floor_robot_.setJointValueTarget(floor_kts2_js_);
    }

    // Try to move to the station with adaptive retries
    bool station_reached = false;
    int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts && !station_reached; attempt++)
    {
        if (attempt > 1)
        {
            RCLCPP_WARN(get_logger(), "Retry %d/%d: Adjusting approach to station", attempt, max_attempts);

            // Adjust the target slightly on subsequent attempts
            std::map<std::string, double> adjusted_js = (station == "kts1") ? floor_kts1_js_ : floor_kts2_js_;

            // Alternate small adjustments to different joints
            if (attempt % 2 == 0)
            {
                adjusted_js["floor_shoulder_pan_joint"] += 0.02 * (attempt / 2);
            }
            else
            {
                adjusted_js["floor_shoulder_lift_joint"] -= 0.02 * ((attempt + 1) / 2);
            }

            floor_robot_.setJointValueTarget(adjusted_js);
        }

        station_reached = FloorRobotMovetoTarget();

        if (!station_reached)
        {
            RCLCPP_WARN(get_logger(), "Failed to reach station on attempt %d/%d", attempt, max_attempts);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    if (!station_reached)
    {
        RCLCPP_ERROR(get_logger(), "Failed to reach kitting station after %d attempts", max_attempts);
        return false;
    }

    // Change gripper to tray gripper if needed
    if (floor_gripper_state_.type != "tray_gripper")
    {
        bool gripper_changed = FloorRobotChangeGripper(station, "trays");
        if (!gripper_changed)
        {
            RCLCPP_ERROR(get_logger(), "Failed to change to tray gripper");
            return false;
        }
    }

    // Move to tray using a multi-stage approach with validation steps
    // Stage 1: Move above tray with good clearance first
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.3, SetRobotOrientation(tray_rotation)));

    bool approach_success = FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);
    if (!approach_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed initial approach above tray, trying backup approach");
        // Try direct joint space move as backup
        floor_robot_.setPoseTarget(waypoints[0]);
        approach_success = FloorRobotMovetoTarget();

        if (!approach_success)
        {
            RCLCPP_ERROR(get_logger(), "Both approach methods failed");
            return false;
        }
    }

    // Wait briefly for system to stabilize
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    // Stage 2: Move to intermediate position halfway down
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.1, SetRobotOrientation(tray_rotation)));

    approach_success = FloorRobotMoveCartesian(waypoints, 0.2, 0.2, true);
    if (!approach_success)
    {
        RCLCPP_WARN(get_logger(), "Failed intermediate approach, attempting direct final approach");
    }
    else
    {
        // Wait briefly for system to stabilize
        rclcpp::sleep_for(std::chrono::milliseconds(150));
    }

    // Stage 3: Final approach to grasp position - slower and more precise
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));

    // Use slower, more precise motion for final grasp positioning
    approach_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true);
    if (!approach_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to move to final grasp position");

        // Try a direct pose target as last resort
        floor_robot_.setPoseTarget(waypoints[0]);
        if (!FloorRobotMovetoTarget())
        {
            RCLCPP_ERROR(get_logger(), "All approach methods failed");
            return false;
        }
    }

    // Enable gripper with improved attachment strategy
    FloorRobotSetGripperState(true);

    // Use WaitForAttach with longer timeout
    FloorRobotWaitForAttach(15.0);

    // Check if attachment was successful
    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "Failed to attach to tray after multiple attempts");
        FloorRobotSetGripperState(false);

        // Move up to clear the area
        waypoints.clear();
        waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                      tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
        FloorRobotMoveCartesian(waypoints, 0.2, 0.2, false);
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Successfully picked kit tray " << tray_id);

    // Add kit tray to planning scene for collision avoidance
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    try
    {
        AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
        floor_robot_.attachObject(tray_name);
        order_planning_scene_objects_.push_back(tray_name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(get_logger(), "Error adding tray to planning scene: %s", e.what());
        // Continue even if planning scene update fails
    }

    // *** CRITICAL SECTION: LIFTING TRAY AFTER PICKUP ***
    // Use a careful multi-stage lifting strategy
    RCLCPP_INFO(get_logger(), "Lifting tray with careful vertical motion");

    // Stage 1: Small initial lift to clear surface
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.1, SetRobotOrientation(tray_rotation)));

    bool lift_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true);
    if (!lift_success)
    {
        RCLCPP_WARN(get_logger(), "Initial lift failed, trying backup method");
    }
    else
    {
        // Wait briefly for stability
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // Stage 2: Complete lift to safe height
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.4, SetRobotOrientation(tray_rotation)));

    lift_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true);
    if (!lift_success)
    {
        RCLCPP_WARN(get_logger(), "Cartesian lift failed, trying to return to station position");

        // If Cartesian lift fails, use joint space motion to return to the safe station position
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(floor_kts2_js_);
        }

        // Use slightly modified joint values to lift higher
        std::map<std::string, double> lift_js = floor_robot_.getNamedTargetValues(
            station == "kts1" ? "kts1_js_" : "kts2_js_");
        lift_js["floor_shoulder_lift_joint"] -= 0.2; // Lift arm higher
        floor_robot_.setJointValueTarget(lift_js);

        FloorRobotMovetoTarget();
    }

    // Wait for stability before moving to AGV
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Move to AGV with improved approach
    RCLCPP_INFO_STREAM(get_logger(), "Moving tray to AGV " << agv_num);

    // First set the rail position but maintain a safe arm configuration
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    // Keep elbow and shoulder in a configuration that maintains height
    floor_robot_.setJointValueTarget("floor_shoulder_lift_joint", -1.0); // Higher position
    floor_robot_.setJointValueTarget("floor_elbow_joint", 1.8);          // Compensated position

    bool agv_approach = FloorRobotMovetoTarget();
    if (!agv_approach)
    {
        RCLCPP_WARN(get_logger(), "Failed to move to AGV approach position, trying alternative");

        // Try a simpler approach with just essential joints
        floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        agv_approach = FloorRobotMovetoTarget();

        if (!agv_approach)
        {
            RCLCPP_ERROR(get_logger(), "Failed to reach AGV position");
            return false;
        }
    }

    // Get AGV tray pose for placement
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    // Validate AGV tray pose
    if (std::isnan(agv_tray_pose.position.x) || std::isnan(agv_tray_pose.position.y) ||
        std::isnan(agv_tray_pose.position.z))
    {
        RCLCPP_ERROR(get_logger(), "Invalid AGV tray pose (contains NaN values)");
        return false;
    }

    // Now do careful Cartesian motion for placement
    // Stage 1: Move above the AGV tray with good clearance
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    bool above_agv_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true);
    if (!above_agv_success)
    {
        RCLCPP_WARN(get_logger(), "Failed Cartesian move above AGV, trying joint space");

        // Try direct joint space move
        floor_robot_.setPoseTarget(waypoints[0]);
        above_agv_success = FloorRobotMovetoTarget();

        if (!above_agv_success)
        {
            RCLCPP_ERROR(get_logger(), "Failed to position above AGV");
            return false;
        }
    }

    // Wait for stability
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    // Stage 2: Intermediate approach to reduce placement distance
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_ + 0.1,
                                  SetRobotOrientation(agv_rotation)));

    bool intermediate_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true);
    if (!intermediate_success)
    {
        RCLCPP_WARN(get_logger(), "Skipping intermediate approach, trying direct placement");
    }
    else
    {
        // Wait for stability
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // Stage 3: Final placement with precise motion
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_,
                                  SetRobotOrientation(agv_rotation)));

    bool place_success = FloorRobotMoveCartesian(waypoints, 0.1, 0.2, true);
    if (!place_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to place tray on AGV precisely");

        // Try direct joint space move as last resort
        floor_robot_.setPoseTarget(waypoints[0]);
        place_success = FloorRobotMovetoTarget();

        if (!place_success)
        {
            RCLCPP_ERROR(get_logger(), "All placement methods failed");
            return false;
        }
    }

    // Release the tray and lock it to the AGV
    FloorRobotSetGripperState(false);

    // Wait to ensure release
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    try
    {
        floor_robot_.detachObject(tray_name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(get_logger(), "Error detaching tray from planning scene: %s", e.what());
        // Continue even if planning scene update fails
    }

    // Lock the tray with retry
    bool lock_success = false;
    for (int lock_attempt = 0; lock_attempt < 3 && !lock_success; lock_attempt++)
    {
        lock_success = LockAGVTray(agv_num);
        if (!lock_success)
        {
            RCLCPP_WARN(get_logger(), "Failed to lock tray to AGV, attempt %d/3", lock_attempt + 1);
            rclcpp::sleep_for(std::chrono::milliseconds(300));
        }
    }

    if (!lock_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to lock tray to AGV after multiple attempts");
    }

    // Stage 4: Safe retreat with multi-stage motion
    // First small vertical retraction
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.1, SetRobotOrientation(agv_rotation)));

    bool initial_retreat = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, false);
    if (initial_retreat)
    {
        // Continue with full retreat
        waypoints.clear();
        waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                      agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));
        bool full_retreat = FloorRobotMoveCartesian(waypoints, 0.1, 0.1, false);

        if (!full_retreat)
        {
            RCLCPP_WARN(get_logger(), "Full retraction failed, trying joint space motion");
            floor_robot_.setNamedTarget("home");
            FloorRobotMovetoTarget();
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Initial retraction failed, trying joint space motion");
        floor_robot_.setNamedTarget("home");
        FloorRobotMovetoTarget();
    }

    RCLCPP_INFO(get_logger(), "Successfully placed tray on AGV %d", agv_num);
    return true;
}

//////////////////////////////////////////////////
bool RobotController::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color]
                                                             << " " << part_types_[part_to_pick.type] << " from the bins");

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    // Check left bins
    for (auto part : left_bins_parts_)
    {
        if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
        {
            part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "left_bins";
            break;
        }
    }
    // Check right bins
    if (!found_part)
    {
        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "right_bins";
                break;
            }
        }
    }
    if (!found_part)
    {
        RCLCPP_INFO(get_logger(), "Unable to locate part in the bins");
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Found part in " << bin_side << " checking gripper type and moving to pick location");

    double part_rotation = GetYaw(part_pose);

    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper")
    {
        std::string station;
        if (part_pose.position.y < 0)
        {
            station = "kts1";
        }
        else
        {
            station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(floor_kts2_js_);
        }
        FloorRobotMovetoTarget();

        FloorRobotChangeGripper(station, "parts");
    }

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0);

    RCLCPP_INFO_STREAM(get_logger(), "Picked Up the " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type] << " from the bins");

    // Add part to planning scene
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

bool RobotController::FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick)
{
    // Validate part availability with proper locking
    {
        std::lock_guard<std::mutex> lock(conveyor_parts_mutex);
        if (conveyor_parts_expected_.empty())
        {
            RCLCPP_INFO(get_logger(), "No parts expected on the conveyor");
            return false;
        }

        bool part_expected = false;
        for (const auto &parts : conveyor_parts_expected_)
        {
            if (parts.part.type == part_to_pick.type && parts.part.color == part_to_pick.color)
            {
                part_expected = true;
                break;
            }
        }

        if (!part_expected)
        {
            RCLCPP_INFO(get_logger(), "Requested part not expected on conveyor");
            return false;
        }
    }

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color]
                                                             << " " << part_types_[part_to_pick.type] << " from the conveyor");

    // Change gripper if needed
    if (floor_gripper_state_.type != "part_gripper")
    {
        // Choose appropriate tool change location based on current position
        std::string station = "kts2"; // Default to kts2
        geometry_msgs::msg::Pose current_pose = floor_robot_.getCurrentPose().pose;

        if (current_pose.position.y < 0)
        {
            station = "kts1";
        }

        floor_robot_.setJointValueTarget(station == "kts1" ? floor_kts1_js_ : floor_kts2_js_);
        if (!FloorRobotMovetoTarget())
        {
            RCLCPP_ERROR(get_logger(), "Failed to move to gripper change position");
            return false;
        }

        if (!FloorRobotChangeGripper(station, "parts"))
        {
            RCLCPP_ERROR(get_logger(), "Failed to change to parts gripper");
            return false;
        }
    }

    RCLCPP_INFO_STREAM(get_logger(), "Moving floor robot to conveyor pick location");

    // Move robot to predefined pick location with improved error handling
    int position_attempts = 0;
    const int max_position_attempts = 3;
    bool position_reached = false;

    while (!position_reached && position_attempts < max_position_attempts)
    {
        position_attempts++;

        try
        {
            floor_robot_.setJointValueTarget(floor_conveyor_js_);
            position_reached = FloorRobotMovetoTarget();

            if (!position_reached)
            {
                RCLCPP_WARN(get_logger(), "Failed to move to conveyor position on attempt %d/%d",
                            position_attempts, max_position_attempts);

                // Slightly modify target for next attempt
                if (position_attempts < max_position_attempts)
                {
                    std::map<std::string, double> modified_js = floor_conveyor_js_;
                    modified_js["floor_shoulder_pan_joint"] += 0.02 * (position_attempts % 2 == 0 ? 1 : -1);
                    floor_robot_.setJointValueTarget(modified_js);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception while moving to conveyor position: %s", e.what());
            if (position_attempts >= max_position_attempts)
            {
                return false;
            }
        }
    }

    if (!position_reached)
    {
        RCLCPP_ERROR(get_logger(), "Failed to reach conveyor position after %d attempts", max_position_attempts);
        return false;
    }

    // Wait for target part with improved detection logic
    rclcpp::Time search_start = now();
    const rclcpp::Duration search_timeout = rclcpp::Duration::from_seconds(30.0);
    bool found_part = false;
    geometry_msgs::msg::Pose part_pose;
    rclcpp::Time detection_time;

    RCLCPP_INFO(get_logger(), "Waiting for part to appear on conveyor (timeout: 30s)");

    while (!found_part && now() - search_start < search_timeout)
    {
        // Lock mutex while searching for parts
        {
            std::lock_guard<std::mutex> lock(conveyor_parts_mutex);

            // First remove any parts that have passed the pick zone to avoid processing stale data
            auto it = conveyor_parts_.begin();
            while (it != conveyor_parts_.end())
            {
                rclcpp::Duration elapsed_time = now() - it->second;
                double current_y = MultiplyPose(conveyor_camera_pose_, it->first.pose).position.y -
                                   (elapsed_time.seconds() * conveyor_speed_);

                if (current_y < -0.5)
                {
                    it = conveyor_parts_.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            // Now find the target part
            for (it = conveyor_parts_.begin(); it != conveyor_parts_.end(); ++it)
            {
                if (it->first.part.type == part_to_pick.type && it->first.part.color == part_to_pick.color)
                {
                    part_pose = MultiplyPose(conveyor_camera_pose_, it->first.pose);
                    detection_time = it->second;

                    // Calculate current part position based on conveyor speed
                    rclcpp::Duration elapsed_time = now() - detection_time;
                    double current_y = part_pose.position.y - (elapsed_time.seconds() * conveyor_speed_);

                    // Check if part is approaching the pick location with sufficient time to prepare
                    if (current_y > 0 && current_y < 2.0)
                    {
                        double time_to_arrival = current_y / conveyor_speed_;

                        // If part will arrive within reasonable time window (1-5 seconds)
                        if (time_to_arrival > 1.0 && time_to_arrival < 5.0)
                        {
                            found_part = true;
                            // Remove this part from tracking list to avoid double-picking
                            conveyor_parts_.erase(it);
                            break;
                        }
                    }
                }
            }
        } // End lock_guard scope

        if (!found_part)
        {
            // Avoid busy waiting with short sleep
            rclcpp::sleep_for(std::chrono::milliseconds(50));

            // Provide feedback every 5 seconds
            if ((now() - search_start).seconds() / 5.0 == static_cast<int>((now() - search_start).seconds() / 5.0))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Waiting for %s %s to arrive on the conveyor (%.1f seconds elapsed)",
                                     part_colors_[part_to_pick.color].c_str(),
                                     part_types_[part_to_pick.type].c_str(),
                                     (now() - search_start).seconds());
            }
        }
    }

    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "No suitable part found on conveyor within timeout period");
        return false;
    }

    // Part found - prepare for interception
    RCLCPP_INFO(get_logger(), "Part detected, preparing interception");

    // Calculate precise intercept timing and position
    double part_rotation = GetYaw(part_pose);
    // Robot position only used for its x-coordinate
    double robot_x = part_pose.position.x; // Align with part's x position
    double robot_y = 0.0;                  // Fixed y position for interception

    // Calculate updated part timing
    rclcpp::Duration elapsed_time = now() - detection_time;
    double current_y = part_pose.position.y - (elapsed_time.seconds() * conveyor_speed_);
    double time_to_arrival = current_y / conveyor_speed_;

    if (current_y <= 0 || time_to_arrival <= 0.5)
    {
        RCLCPP_WARN(get_logger(), "Part too close or has passed the pick location (y=%.3f, ETA=%.2fs)",
                    current_y, time_to_arrival);
        return false;
    }

    // Pre-position robot at calculated intercept point with fixed z-offset
    double pick_z = part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(robot_x, robot_y, pick_z + 0.15, SetRobotOrientation(part_rotation)));

    if (!FloorRobotMoveCartesian(waypoints, 0.8, 0.8, true))
    {
        RCLCPP_ERROR(get_logger(), "Failed to move to pre-pick position");
        return false;
    }

    // Wait for optimal interception time
    // Recalculate timing after robot has moved into position
    elapsed_time = now() - detection_time;
    current_y = part_pose.position.y - (elapsed_time.seconds() * conveyor_speed_);
    time_to_arrival = current_y / conveyor_speed_;

    // Plan final approach trajectory
    waypoints.clear();

    // Target position for the actual pick
    geometry_msgs::msg::Pose pick_pose = BuildPose(
        robot_x, robot_y, pick_z, SetRobotOrientation(part_rotation));
    waypoints.push_back(pick_pose);

    // Plan but don't execute yet
    auto trajectory_result = FloorRobotPlanCartesian(waypoints, 0.8, 0.8, true);
    if (!trajectory_result.first)
    {
        RCLCPP_ERROR(get_logger(), "Failed to plan trajectory to pick part");
        return false;
    }

    // Calculate exact time to execute the trajectory
    double trajectory_duration_sec = trajectory_result.second.joint_trajectory.points.back().time_from_start.sec +
                                     trajectory_result.second.joint_trajectory.points.back().time_from_start.nanosec / 1e9;
    double wait_time = time_to_arrival - trajectory_duration_sec - 0.15; // Buffer for gripper activation

    if (wait_time < 0)
    {
        RCLCPP_WARN(get_logger(), "Part moving too fast to intercept (%.2f sec behind)", -wait_time);
        return false;
    }

    if (wait_time > 0)
    {
        RCLCPP_INFO(get_logger(), "Waiting %.2f seconds for optimal interception", wait_time);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time * 1000)));
    }

    // Turn on gripper just before movement
    FloorRobotSetGripperState(true);

    // Execute final approach
    bool execution_success = static_cast<bool>(floor_robot_.execute(trajectory_result.second));
    if (!execution_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to execute pick trajectory");
        FloorRobotSetGripperState(false);
        return false;
    }

    // Check for successful attachment with timeout
    rclcpp::Time attach_start = now();
    rclcpp::Duration attach_timeout = rclcpp::Duration::from_seconds(1.0);
    bool attached = false;

    while (now() - attach_start < attach_timeout)
    {
        if (floor_gripper_state_.attached)
        {
            attached = true;
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    if (!attached)
    {
        RCLCPP_ERROR(get_logger(), "Failed to attach to part");
        FloorRobotSetGripperState(false);
        return false;
    }

    // Successfully picked part
    RCLCPP_INFO_STREAM(get_logger(), "Successfully picked " << part_colors_[part_to_pick.color]
                                                            << " " << part_types_[part_to_pick.type] << " from conveyor");

    // Update planning scene with error handling
    part_pose.position.y = robot_y; // Update with actual pick position
    std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];

    try
    {
        AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
        floor_robot_.attachObject(part_name);
        order_planning_scene_objects_.push_back(part_name);
        floor_robot_attached_part_ = part_to_pick;
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(get_logger(), "Error updating planning scene: %s", e.what());
        // Continue even if planning scene update fails
    }

    // Move to safe position after pick
    waypoints.clear();
    waypoints.push_back(BuildPose(robot_x, robot_y, pick_z + 0.2, SetRobotOrientation(part_rotation)));

    if (!FloorRobotMoveCartesian(waypoints, 0.3, 0.3, false))
    {
        RCLCPP_WARN(get_logger(), "Failed post-pick retraction, attempting recovery");

        // Fall back to joint-space move
        floor_robot_.setNamedTarget("home");
        FloorRobotMovetoTarget();
    }

    return true;
}

bool RobotController::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Placing the " << part_colors_[floor_robot_attached_part_.color]
                                                    << " " << part_types_[floor_robot_attached_part_.type]
                                                    << " on AGV " << agv_num << " in quadrant " << quadrant);

    // Validate inputs
    if (agv_num < 1 || agv_num > 4)
    {
        RCLCPP_ERROR(get_logger(), "Invalid AGV number: %d", agv_num);
        return false;
    }

    if (quadrant < 1 || quadrant > 4)
    {
        RCLCPP_ERROR(get_logger(), "Invalid quadrant number: %d", quadrant);
        return false;
    }

    // Move to AGV with retry
    int max_move_attempts = 3;
    bool move_success = false;

    for (int attempt = 1; attempt <= max_move_attempts && !move_success; attempt++)
    {
        floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

        if (FloorRobotMovetoTarget())
        {
            move_success = true;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to move to AGV %d on attempt %d", agv_num, attempt);

            if (attempt < max_move_attempts)
            {
                // Try adjusting the target slightly
                floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)] + 0.01);
                floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0.01);
                FloorRobotMovetoTarget();
            }
        }
    }

    if (!move_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to move to AGV %d after %d attempts", agv_num, max_move_attempts);
        return false;
    }

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    // Validate tray pose
    if (std::isnan(agv_tray_pose.position.x) || std::isnan(agv_tray_pose.position.y) || std::isnan(agv_tray_pose.position.z))
    {
        RCLCPP_ERROR(get_logger(), "Invalid AGV tray pose (contains NaN)");
        return false;
    }

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    // Approach and place with improved error handling
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // First move to safe position above drop location
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.3, 0.3, true))
    {
        RCLCPP_ERROR(get_logger(), "Failed to move to position above drop location");
        return false;
    }

    // Then move down to drop position
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] +
                                      kit_tray_thickness_ + drop_height_,
                                  SetRobotOrientation(0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.1, 0.1, true))
    {
        RCLCPP_ERROR(get_logger(), "Failed to move to drop position");
        return false;
    }

    // Drop part with proper detachment
    FloorRobotSetGripperState(false);

    // Wait to ensure part is released
    rclcpp::sleep_for(std::chrono::milliseconds(250));

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

    // Move up to safe position
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.2,
                                  SetRobotOrientation(0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.3, 0.3, false))
    {
        RCLCPP_WARN(get_logger(), "Failed to move up after placing part, attempting recovery");
        floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        FloorRobotMovetoTarget();
    }

    RCLCPP_INFO(get_logger(), "Successfully placed part on AGV %d in quadrant %d", agv_num, quadrant);
    return true;
}

void RobotController::CeilingRobotSendHome()
{
    // Move ceiling robot to home joint state
    RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to home position");
    ceiling_robot_.setNamedTarget("home");
    CeilingRobotMovetoTarget();
}

bool RobotController::CeilingRobotSetGripperState(bool enable)
{
    if (ceiling_gripper_state_.enabled == enable)
    {
        if (ceiling_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto future = ceiling_robot_gripper_enable_->async_send_request(request);
    future.wait();

    if (!future.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

void RobotController::CeilingRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

    while (!ceiling_gripper_state_.attached)
    {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

        usleep(500);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

bool RobotController::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
    // Wait for part to be attached
    rclcpp::Time start = now();

    auto rotation = GetYaw(FrameWorldPose("as" + std::to_string(station) + "_insert_frame"));

    bool assembled = false;

    double R[3][3] = {cos(rotation), -sin(rotation), 0,
                      sin(rotation), cos(rotation), 0,
                      0, 0, 1};

    double dx = R[0][0] * part.install_direction.x + R[0][1] * part.install_direction.y + R[0][2] * part.install_direction.z;
    double dy = R[1][0] * part.install_direction.x + R[1][1] * part.install_direction.y + R[1][2] * part.install_direction.z;
    double dz = R[2][0] * part.install_direction.x + R[2][1] * part.install_direction.y + R[2][2] * part.install_direction.z;

    geometry_msgs::msg::Pose pose = ceiling_robot_.getCurrentPose().pose;

    pose.position.x += dx * (assembly_offset_ + 0.005);
    pose.position.y += dy * (assembly_offset_ + 0.005);
    pose.position.z += dz * (assembly_offset_ + 0.005);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose);

    auto ret = CeilingRobotPlanCartesian(waypoints, 0.01, 0.01, false);

    if (ret.first)
    {
        ceiling_robot_.asyncExecute(ret.second);
    }
    else
    {
        return false;
    }

    while (!assembled)
    {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

        // Check if part is assembled
        switch (part.part.type)
        {
        case ariac_msgs::msg::Part::BATTERY:
            assembled = assembly_station_states_[station].battery_attached;
            break;
        case ariac_msgs::msg::Part::PUMP:
            assembled = assembly_station_states_[station].pump_attached;
            break;
        case ariac_msgs::msg::Part::SENSOR:
            assembled = assembly_station_states_[station].sensor_attached;
            break;
        case ariac_msgs::msg::Part::REGULATOR:
            assembled = assembly_station_states_[station].regulator_attached;
            break;
        default:
            RCLCPP_WARN(get_logger(), "Not a valid part type");
            return false;
        }
    }

    ceiling_robot_.stop();

    RCLCPP_INFO(get_logger(), "Part is assembled");

    return true;
}

bool RobotController::CeilingRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(ceiling_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(ceiling_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool RobotController::CeilingRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
    rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> RobotController::CeilingRobotPlanCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return std::make_pair(false, trajectory);
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
    rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return std::make_pair(true, trajectory);
}

bool RobotController::CeilingRobotMoveToAssemblyStation(int station)
{
    RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to assembly station " << station);
    switch (station)
    {
    case 1:
        ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
        break;
    case 2:
        ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
        break;
    case 3:
        ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
        break;
    case 4:
        ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
        break;
    default:
        RCLCPP_WARN(get_logger(), "Not a valid assembly station");
        return false;
    }

    return CeilingRobotMovetoTarget();
}

bool RobotController::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
    RCLCPP_INFO_STREAM(get_logger(), "Determining waypoints to pick " << part_types_[part.part.type]);
    double part_rotation = GetYaw(part.pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;

    double dx = 0;
    double dy = 0;

    if (part.part.type == ariac_msgs::msg::Part::BATTERY)
    {
        dx = battery_grip_offset_ * cos(part_rotation);
        dy = battery_grip_offset_ * sin(part_rotation);
    }

    waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                  part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                  part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));

    RCLCPP_INFO_STREAM(get_logger(), "Moving ceiling robot to pick " << part_types_[part.part.type]);
    CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, false);

    CeilingRobotSetGripperState(true);

    CeilingRobotWaitForAttach(5.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
    AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
    ceiling_robot_.attachObject(part_name);
    ceiling_robot_attached_part_ = part.part;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part.pose.position.x, part.pose.position.y,
                                  part.pose.position.z + 0.3,
                                  SetRobotOrientation(part_rotation)));

    CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    return true;
}

bool RobotController::CompleteOrders()
{
    // Wait for first order to be published
    while (orders_.size() == 0)
    {
    }

    bool success{false};
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            success = false;
            break;
        }

        if (orders_.size() == 0)
        {
            // FIXME This needs some work
            if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
            {
                // wait for more orders
                RCLCPP_INFO(get_logger(), "Waiting for orders...");
                while (orders_.size() == 0)
                {
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Completed all orders");
                success = true;
                break;
            }
        }

        current_order_ = orders_.front();
        orders_.erase(orders_.begin());
        // int kitting_agv_num = -1;

        bool task_completed{false};
        if (current_order_.type == ariac_msgs::msg::Order::KITTING)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Current order is: " << current_order_.id << " of type KITTING");
            task_completed = RobotController::CompleteKittingTask(current_order_.kitting_task);
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Task unknown");
        }

        if (task_completed == true)
        {
            RobotController::SubmitOrder(current_order_.id);
        }
    }
    return success;
}

bool RobotController::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{
    RCLCPP_INFO(get_logger(), "Starting kitting task");

    // Validate inputs
    if (task.agv_number < 1 || task.agv_number > 4)
    {
        RCLCPP_ERROR(get_logger(), "Invalid AGV number: %d", task.agv_number);
        return false;
    }

    if (task.tray_id < 0)
    {
        RCLCPP_ERROR(get_logger(), "Invalid tray ID: %d", task.tray_id);
        return false;
    }

    // Start timing for diagnostics
    rclcpp::Time task_start = now();

    // Move floor robot to safe starting position
    FloorRobotSendHome();

    // Ensure AGV is at kitting station
    if (agv_locations_[task.agv_number] != ariac_msgs::msg::AGVStatus::KITTING)
    {
        RCLCPP_INFO(get_logger(), "Moving AGV %d to kitting station", task.agv_number);
        if (!MoveAGV(task.agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING))
        {
            RCLCPP_ERROR(get_logger(), "Failed to move AGV to kitting station");
            return false;
        }
    }

    // Pick and place tray with retry
    int tray_attempts = 0;
    bool tray_success = false;

    while (!tray_success && tray_attempts < 3)
    {
        tray_attempts++;
        tray_success = FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

        if (!tray_success)
        {
            RCLCPP_WARN(get_logger(), "Failed to pick and place tray, attempt %d/3", tray_attempts);

            if (tray_attempts < 3)
            {
                // Return to home and try again
                FloorRobotSendHome();
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    if (!tray_success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to pick and place tray after 3 attempts");
        return false;
    }

    // Process each part in the kit
    for (auto kit_part : task.parts)
    {
        // Validate quadrant
        if (kit_part.quadrant < 1 || kit_part.quadrant > 4)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quadrant %d for part", kit_part.quadrant);
            continue; // Skip this part but try to complete others
        }

        // Try to pick from bins first
        bool found = FloorRobotPickBinPart(kit_part.part);

        // If not in bins, try conveyor
        if (!found)
        {
            RCLCPP_INFO(get_logger(), "Part not found in bins, trying conveyor");
            found = FloorRobotPickConveyorPart(kit_part.part);
        }

        if (!found)
        {
            RCLCPP_ERROR(get_logger(), "Failed to find part %s %s in bins or conveyor",
                         part_colors_[kit_part.part.color].c_str(),
                         part_types_[kit_part.part.type].c_str());
            continue; // Skip this part but try to complete others
        }

        // Place part on tray
        if (!FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant))
        {
            RCLCPP_ERROR(get_logger(), "Failed to place part on kit tray");
            continue; // Skip this part but try to complete others
        }
    }

    // Check quality
    RCLCPP_INFO(get_logger(), "Checking quality of kitting task");
    auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request->order_id = current_order_.id;

    bool quality_success = false;
    int quality_attempts = 0;

    while (!quality_success && quality_attempts < 3)
    {
        quality_attempts++;

        try
        {
            auto future = quality_checker_->async_send_request(request);

            // Use a timeout for the service call
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            {
                auto result = future.get();

                if (result->all_passed)
                {
                    quality_success = true;
                    RCLCPP_INFO(get_logger(), "Quality check passed");
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Quality check failed on attempt %d/3", quality_attempts);

                    // Check if there are incorrect trays and if we're under the max attempts
                    if (quality_attempts < 3 && result->incorrect_tray)
                    {
                        RCLCPP_INFO(get_logger(), "Detected incorrect tray placement, attempting to fix");

                        // Return to home position
                        FloorRobotSendHome();

                        // Try to fix tray issues (would require more complex logic to actually implement)
                        // For now, just try to place the tray again
                        if (FloorRobotPickandPlaceTray(task.tray_id, task.agv_number))
                        {
                            RCLCPP_INFO(get_logger(), "Successfully replaced tray");
                        }
                        else
                        {
                            RCLCPP_ERROR(get_logger(), "Failed to replace tray");
                        }
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Quality check service timed out");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error calling quality check service: %s", e.what());
        }

        if (!quality_success && quality_attempts < 3)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    if (!quality_success)
    {
        RCLCPP_WARN(get_logger(), "Quality check failed after maximum attempts, proceeding anyway");
    }

    // Clean up planning scene
    planning_scene_.removeCollisionObjects(order_planning_scene_objects_);
    order_planning_scene_objects_.clear();

    // Move AGV to destination
    RCLCPP_INFO(get_logger(), "Moving AGV %d to destination %d",
                task.agv_number, task.destination);

    if (!MoveAGV(task.agv_number, task.destination))
    {
        RCLCPP_ERROR(get_logger(), "Failed to move AGV to destination");
        return false;
    }

    // Log completion time
    double task_duration = (now() - task_start).seconds();
    RCLCPP_INFO(get_logger(), "Completed kitting task in %.2f seconds", task_duration);

    return true;
}

bool RobotController::StartCompetition()
{
    // Don't wait in a blocking loop - just check if ready
    if (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
    {
        RCLCPP_INFO(get_logger(), "Competition not ready yet");
        return false;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), BOLD + ORANGE << "Competition is ready - starting competition" << RESET);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    std::string srv_name = "/ariac/start_competition";
    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    // Wait for service to be available with timeout
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(get_logger(), "Start competition service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    // Use a timeout for the future
    std::future_status status = future.wait_for(std::chrono::seconds(3));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Start competition service call timed out");
        return false;
    }

    return future.get()->success;
}

bool RobotController::EndCompetition()
{
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/end_competition";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    RCLCPP_INFO(get_logger(), "Ending competition.");

    auto future = client->async_send_request(request);
    future.wait();

    return future.get()->success;
}

bool RobotController::SubmitOrder(std::string order_id)
{
    RCLCPP_INFO_STREAM(get_logger(), "Submitting order " << order_id);
    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
    std::string srv_name = "/ariac/submit_order";
    client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id;

    auto future = client->async_send_request(request);
    future.wait();

    return future.get()->success;
}

bool RobotController::LockAGVTray(int agv_num)
{
    RCLCPP_INFO_STREAM(get_logger(), "Locking Tray to AGV" << agv_num);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future = client->async_send_request(request);
    future.wait();

    return future.get()->success;
}

bool RobotController::UnlockAGVTray(int agv_num)
{
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_unlock_tray";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future = client->async_send_request(request);
    future.wait();

    return future.get()->success;
}

bool RobotController::MoveAGV(int agv_num, int destination)
{
    RCLCPP_INFO_STREAM(get_logger(), "Moving AGV" << agv_num << " to " << agv_destination_[destination]);
    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

    std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

    client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = destination;

    auto future = client->async_send_request(request);

    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(20);

    rclcpp::Time start = get_clock()->now();

    while (get_clock()->now() - start < timeout)
    {
        if (agv_locations_[agv_num] == destination)
        {
            return true;
        }
    }

    RCLCPP_INFO_STREAM(get_logger(), "Unable to move AGV" << agv_num << " to " << agv_destination_[destination]);
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}