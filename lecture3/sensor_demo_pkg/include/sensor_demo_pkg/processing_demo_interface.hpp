#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class ProcessingDemoInterface : public rclcpp::Node
{
public:
    ProcessingDemoInterface()
        : Node("processing_demo")
    {
        // Callback group for sensor subscribers
        reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = reentrant_group_;

        // LiDAR Subscriber
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ProcessingDemoInterface::lidar_callback, this, std::placeholders::_1), options);

        // Camera Subscriber
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ProcessingDemoInterface::camera_callback, this, std::placeholders::_1), options);

        // Temperature Subscriber
        temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
            "/temperature", 10, std::bind(&ProcessingDemoInterface::temperature_callback, this, std::placeholders::_1), options);

        RCLCPP_INFO(this->get_logger(), "processing_demo node started");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;

    // Callback group
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
};
