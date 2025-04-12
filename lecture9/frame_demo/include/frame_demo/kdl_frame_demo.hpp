#pragma once

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kdl/frames.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace frame_demo {

class KDLFrameDemo : public rclcpp::Node {
   public:
    // Constructor
    explicit KDLFrameDemo();

   private:
    // Callback functions
    void find_part_callback();
    geometry_msgs::msg::Pose compute_part_pose_in_world(
        const geometry_msgs::msg::Pose& part_pose_in_camera,
        const geometry_msgs::msg::Pose& camera_pose_in_world);
    void left_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void right_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    // Parameters
    bool use_sim_time_;
    bool use_kdl_frame_;  // New parameter to control node activation

    // Part detection state
    std::vector<ariac_msgs::msg::PartPose> left_bin_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bin_parts_;
    bool found_purple_pump_;

    // Target part specifications
    uint8_t find_part_color_;
    uint8_t find_part_type_;

    // Camera and part poses
    std::optional<geometry_msgs::msg::Pose> right_bins_camera_pose_in_world_;
    std::optional<geometry_msgs::msg::Pose> left_bins_camera_pose_in_world_;
    std::optional<geometry_msgs::msg::Pose> part_pose_in_world_;

    // ROS2 communication
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    rclcpp::TimerBase::SharedPtr find_part_timer_;
};

}  // namespace frame_demo