#include "frame_demo/kdl_frame_demo.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <iomanip>
#include <kdl/frames.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace {
// Helper function to convert quaternion to Euler angles
std::tuple<double, double, double> euler_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
    Eigen::Vector3d euler = eigen_q.toRotationMatrix().eulerAngles(0, 1, 2);
    return std::make_tuple(euler[0], euler[1], euler[2]);
}

// Helper function to convert geometry_msgs::msg::Quaternion to KDL::Rotation
KDL::Rotation ros_quaternion_to_kdl_rotation(const geometry_msgs::msg::Quaternion& q) {
    return KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
}

// Helper function to convert KDL::Rotation to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion kdl_rotation_to_ros_quaternion(const KDL::Rotation& rot) {
    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}
}  // namespace

namespace frame_demo {

KDLFrameDemo::KDLFrameDemo()
    : Node("kdl_frame_demo"),
      found_purple_pump_(false),
      find_part_color_(ariac_msgs::msg::Part::PURPLE),
      find_part_type_(ariac_msgs::msg::Part::PUMP) {
    // Set parameters
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", true);
    }

    // Declare the use_kdl_frame parameter with default value false
    this->declare_parameter("use_kdl_frame", false);

    // Get parameter values
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    use_kdl_frame_ = this->get_parameter("use_kdl_frame").as_bool();

    // If use_kdl_frame is false, return early without initializing anything
    if (!use_kdl_frame_) {
        RCLCPP_INFO(this->get_logger(), "KDLFrameDemo is disabled. Set 'use_kdl_frame' to true to enable it.");
        return;
    }

    // Create subscription for left bins camera
    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image",
        rclcpp::SensorDataQoS(),
        std::bind(&KDLFrameDemo::left_bins_camera_callback, this, _1));

    // Create subscription for right bins camera
    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image",
        rclcpp::SensorDataQoS(),
        std::bind(&KDLFrameDemo::right_bins_camera_callback, this, _1));

    // Create timer for finding parts
    find_part_timer_ = this->create_wall_timer(
        50ms, std::bind(&KDLFrameDemo::find_part_callback, this));

    RCLCPP_INFO(this->get_logger(), "KDL frame demo started");
}

void KDLFrameDemo::find_part_callback() {
    // Return early if the node is disabled
    if (!use_kdl_frame_) {
        return;
    }

    if (!found_purple_pump_) {
        RCLCPP_INFO(this->get_logger(), "Searching...");

        // Search for purple pump in left bin
        for (const auto& part_pose : left_bin_parts_) {
            if (part_pose.part.color == find_part_color_ &&
                part_pose.part.type == find_part_type_) {
                found_purple_pump_ = true;

                // Check if left bins camera pose is available
                if (left_bins_camera_pose_in_world_.has_value()) {
                    part_pose_in_world_ = compute_part_pose_in_world(
                        part_pose.pose,
                        left_bins_camera_pose_in_world_.value());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Left bins camera pose not available");
                }
                break;
            }
        }

        // If not found in left bin, search in right bin
        if (!found_purple_pump_) {
            for (const auto& part_pose : right_bin_parts_) {
                if (part_pose.part.color == find_part_color_ &&
                    part_pose.part.type == find_part_type_) {
                    found_purple_pump_ = true;

                    // Check if right bins camera pose is available
                    if (right_bins_camera_pose_in_world_.has_value()) {
                        part_pose_in_world_ = compute_part_pose_in_world(
                            part_pose.pose,
                            right_bins_camera_pose_in_world_.value());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Right bins camera pose not available");
                    }
                    break;
                }
            }
        }
    }
}

geometry_msgs::msg::Pose KDLFrameDemo::compute_part_pose_in_world(
    const geometry_msgs::msg::Pose& part_pose_in_camera,
    const geometry_msgs::msg::Pose& camera_pose_in_world) {
    // 1. Create KDL Frame for camera-to-world transformation
    KDL::Frame kdl_camera_world(
        ros_quaternion_to_kdl_rotation(camera_pose_in_world.orientation),
        KDL::Vector(
            camera_pose_in_world.position.x,
            camera_pose_in_world.position.y,
            camera_pose_in_world.position.z));

    // 2. Create KDL Frame for part-to-camera transformation
    KDL::Frame kdl_part_camera(
        ros_quaternion_to_kdl_rotation(part_pose_in_camera.orientation),
        KDL::Vector(
            part_pose_in_camera.position.x,
            part_pose_in_camera.position.y,
            part_pose_in_camera.position.z));

    // 3. Compute part-to-world transformation by composing the two frames
    KDL::Frame kdl_part_world = kdl_camera_world * kdl_part_camera;

    // 4. Convert the KDL frame back to a Pose message
    geometry_msgs::msg::Pose pose;
    pose.position.x = kdl_part_world.p.x();
    pose.position.y = kdl_part_world.p.y();
    pose.position.z = kdl_part_world.p.z();
    pose.orientation = kdl_rotation_to_ros_quaternion(kdl_part_world.M);

    // Log the resulting pose information
    auto [roll, pitch, yaw] = euler_from_quaternion(pose.orientation);

    std::stringstream output;
    output << "\n"
           << std::string(50, '=') << "\n";
    output << "Part position in world frame: \n";
    output << " x: " << std::fixed << std::setprecision(4) << pose.position.x;
    output << ", y: " << std::fixed << std::setprecision(4) << pose.position.y;
    output << ", z: " << std::fixed << std::setprecision(4) << pose.position.z << "\n";
    output << "Part orientation in world frame: \n";
    output << " roll: " << std::fixed << std::setprecision(4) << roll;
    output << ", pitch: " << std::fixed << std::setprecision(4) << pitch;
    output << ", yaw: " << std::fixed << std::setprecision(4) << yaw << "\n";
    output << std::string(50, '=') << "\n";

    RCLCPP_INFO(this->get_logger(), "%s", output.str().c_str());

    return pose;
}

void KDLFrameDemo::left_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    // Return early if the node is disabled
    if (!use_kdl_frame_) {
        return;
    }

    left_bin_parts_.clear();
    if (msg->part_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No parts detected in left bins");
        return;
    }

    if (!left_bins_camera_pose_in_world_.has_value()) {
        left_bins_camera_pose_in_world_ = msg->sensor_pose;
    }

    for (const auto& part_pose : msg->part_poses) {
        left_bin_parts_.push_back(part_pose);
    }
}

void KDLFrameDemo::right_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    // Return early if the node is disabled
    if (!use_kdl_frame_) {
        return;
    }

    right_bin_parts_.clear();
    if (msg->part_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No parts detected in right bins");
        return;
    }

    if (!right_bins_camera_pose_in_world_.has_value()) {
        right_bins_camera_pose_in_world_ = msg->sensor_pose;
    }

    for (const auto& part_pose : msg->part_poses) {
        right_bin_parts_.push_back(part_pose);
    }
}

}  // namespace frame_demo