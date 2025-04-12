#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace frame_demo {

class BroadcasterDemo : public rclcpp::Node {
   public:
    // Constructor
    explicit BroadcasterDemo();

   private:
    // Callback functions
    void find_part_callback();
    void broadcast();
    void left_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void right_bins_camera_callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void generate_transform(const std::string& parent, const std::string& child, const geometry_msgs::msg::Pose& pose);
    void listener_cb();

    // Parameters
    bool use_sim_time_;
    bool use_broadcaster_listener_;  // New parameter to control node activation

    // Part detection state
    std::vector<ariac_msgs::msg::PartPose> left_bin_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bin_parts_;
    bool found_purple_pump_;
    std::optional<std::string> part_parent_frame_;
    std::string part_frame_;
    std::optional<geometry_msgs::msg::Pose> part_pose_;

    // Target part specifications
    uint8_t find_part_color_;
    uint8_t find_part_type_;

    // ROS2 communication
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    rclcpp::TimerBase::SharedPtr find_part_timer_;

    // Transform broadcasting
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> transforms_;

    // Transform listening
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr listener_timer_;
};

}  // namespace frame_demo