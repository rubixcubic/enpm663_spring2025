#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <random>

class TemperatureDemoInterface : public rclcpp::Node
{
public:
    TemperatureDemoInterface()
        : Node("temperature_demo")
    {
        // Create a publisher for the Temperature message
        publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
        // Create a timer to periodically publish messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // Publish every 1 second
            std::bind(&TemperatureDemoInterface::publish_temperature_data, this));
        RCLCPP_INFO(this->get_logger(), "temperature_demo node started");
    }

private:
    void publish_temperature_data();

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

