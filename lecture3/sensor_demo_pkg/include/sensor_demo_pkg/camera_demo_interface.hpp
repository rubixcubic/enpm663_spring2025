#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <vector>
#include <random>
#include <chrono>
#include <cstring>

class CameraDemoInterface : public rclcpp::Node
{
public:
    CameraDemoInterface()
        : Node("camera_demo")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&CameraDemoInterface::publish_camera_data, this));
        RCLCPP_INFO(this->get_logger(), "camera_demo node started");
    }

private:
    void publish_camera_data();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


