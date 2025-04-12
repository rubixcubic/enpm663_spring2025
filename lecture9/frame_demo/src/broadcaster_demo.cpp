#include "frame_demo/broadcaster_demo.hpp"

#include <tf2/exceptions.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace frame_demo {

BroadcasterDemo::BroadcasterDemo()
    : Node("broadcaster_demo"),
      found_purple_pump_(false),
      part_frame_("purple_pump_1"),
      find_part_color_(ariac_msgs::msg::Part::PURPLE),
      find_part_type_(ariac_msgs::msg::Part::PUMP) {
    // Set up parameters
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", true);
    }

    // Declare the use_broadcaster_listener parameter with default value false
    this->declare_parameter("use_broadcaster_listener", false);

    // Get parameter values
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    use_broadcaster_listener_ = this->get_parameter("use_broadcaster_listener").as_bool();

    // If use_broadcaster_listener is false, return early without initializing anything
    if (!use_broadcaster_listener_) {
        RCLCPP_INFO(this->get_logger(), "BroadcasterDemo is disabled. Set 'use_broadcaster_listener' to true to enable it.");
        return;
    }

    // Create subscription for left bins camera
    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image",
        rclcpp::SensorDataQoS(),
        std::bind(&BroadcasterDemo::left_bins_camera_callback, this, _1));

    // Create subscription for right bins camera
    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image",
        rclcpp::SensorDataQoS(),
        std::bind(&BroadcasterDemo::right_bins_camera_callback, this, _1));

    // Create timer for finding parts
    find_part_timer_ = this->create_wall_timer(
        50ms, std::bind(&BroadcasterDemo::find_part_callback, this));

    // Create transform broadcaster
    tf_dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Create transform listener components
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    listener_timer_ = this->create_wall_timer(
        500ms, std::bind(&BroadcasterDemo::listener_cb, this));

    RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");
}

void BroadcasterDemo::find_part_callback() {
    // Return early if the node is disabled
    if (!use_broadcaster_listener_) {
        return;
    }

    if (!found_purple_pump_) {
        RCLCPP_INFO(this->get_logger(), "Searching...");

        // Search for purple pump in left bin
        for (const auto& part_pose : left_bin_parts_) {
            if (part_pose.part.color == find_part_color_ &&
                part_pose.part.type == find_part_type_) {
                part_parent_frame_ = "left_bins_camera_frame";
                part_pose_ = part_pose.pose;
                found_purple_pump_ = true;
                generate_transform(part_parent_frame_.value(), part_frame_, part_pose_.value());
                break;
            }
        }

        // If not found in left bin, search in right bin
        if (!found_purple_pump_) {
            for (const auto& part_pose : right_bin_parts_) {
                if (part_pose.part.color == find_part_color_ &&
                    part_pose.part.type == find_part_type_) {
                    part_parent_frame_ = "right_bins_camera_frame";
                    part_pose_ = part_pose.pose;
                    found_purple_pump_ = true;
                    generate_transform(part_parent_frame_.value(), part_frame_, part_pose_.value());
                    break;
                }
            }
        }
    } else {
        // If purple pump was already found, broadcast its transform
        broadcast();
    }
}

void BroadcasterDemo::broadcast() {
    // Return early if the node is disabled
    if (!use_broadcaster_listener_ || !tf_dynamic_broadcaster_) {
        return;
    }

    tf_dynamic_broadcaster_->sendTransform(transforms_);
}

void BroadcasterDemo::left_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    // Return early if the node is disabled
    if (!use_broadcaster_listener_) {
        return;
    }

    left_bin_parts_.clear();
    if (msg->part_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No parts detected in left bins");
        return;
    }

    for (const auto& part_pose : msg->part_poses) {
        left_bin_parts_.push_back(part_pose);
    }
}

void BroadcasterDemo::right_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    // Return early if the node is disabled
    if (!use_broadcaster_listener_) {
        return;
    }

    right_bin_parts_.clear();
    if (msg->part_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No parts detected in right bins");
        return;
    }

    for (const auto& part_pose : msg->part_poses) {
        right_bin_parts_.push_back(part_pose);
    }
}

void BroadcasterDemo::generate_transform(
    const std::string& parent,
    const std::string& child,
    const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set header information
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = parent;
    transform_stamped.child_frame_id = child;

    // Set translation from pose position
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;

    // Set rotation from pose orientation
    transform_stamped.transform.rotation.x = pose.orientation.x;
    transform_stamped.transform.rotation.y = pose.orientation.y;
    transform_stamped.transform.rotation.z = pose.orientation.z;
    transform_stamped.transform.rotation.w = pose.orientation.w;

    // Add transform to list
    transforms_.push_back(transform_stamped);
}

void BroadcasterDemo::listener_cb() {
    // Return early if the node is disabled
    if (!use_broadcaster_listener_ || !tf_buffer_) {
        return;
    }

    try {
        if (!part_parent_frame_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "Part parent frame is not set.");
            return;
        }

        // Lookup transform between world frame and part frame
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform("world", part_frame_, tf2::TimePointZero);

        // Log the transform information
        RCLCPP_INFO(
            this->get_logger(),
            "Transform between world and %s: \n"
            "translation: [%f, %f, %f] \n"
            "rotation: [%f, %f, %f, %f]",
            part_frame_.c_str(),
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not get transform between world and %s: %s",
            part_frame_.c_str(), ex.what());
    }
}

}  // namespace frame_demo