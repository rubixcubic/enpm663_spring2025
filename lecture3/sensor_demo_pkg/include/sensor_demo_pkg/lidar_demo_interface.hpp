#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <random>
#include <cmath>

class LidarDemoInterface : public rclcpp::Node
{
public:
    LidarDemoInterface()
        : Node("lidar_demo")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&LidarDemoInterface::publish_lidar_data, this));
        RCLCPP_INFO(this->get_logger(), "lidar_demo node started");
    }

private:
    void publish_lidar_data();

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
