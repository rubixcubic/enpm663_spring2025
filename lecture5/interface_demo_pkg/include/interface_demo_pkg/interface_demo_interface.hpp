#pragma once

#include "rclcpp/rclcpp.hpp"
#include "enpm663_interfaces/msg/day_time_info.hpp"
#include <random>
#include <chrono>

class InterfaceDemoInterface : public rclcpp::Node
{
public:
    InterfaceDemoInterface(const std::string & node_name)
        : Node(node_name)
    {
        // Create publisher
        _pub = this->create_publisher<enpm663_interfaces::msg::DayTimeInfo>("day_time_info", 10);

        // Create timer to call the timer callback every 2 seconds
        pub_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&InterfaceDemoInterface::timer_callback, this));

        // Initialize the time_of_day_list_ and day_of_week_list_
        time_of_day_list_ = {enpm663_interfaces::msg::DayTimeInfo::DAY, enpm663_interfaces::msg::DayTimeInfo::NIGHT};
        day_of_week_list_ = {
            enpm663_interfaces::msg::DayTimeInfo::MONDAY,
            enpm663_interfaces::msg::DayTimeInfo::TUESDAY,
            enpm663_interfaces::msg::DayTimeInfo::WEDNESDAY,
            enpm663_interfaces::msg::DayTimeInfo::THURSDAY,
            enpm663_interfaces::msg::DayTimeInfo::FRIDAY,
            enpm663_interfaces::msg::DayTimeInfo::SATURDAY,
            enpm663_interfaces::msg::DayTimeInfo::SUNDAY
        };

        // Log node initialization
        RCLCPP_INFO(this->get_logger(), "%s initialized", node_name.c_str());
    }

private:
    // Timer callback function
    void timer_callback();

    rclcpp::Publisher<enpm663_interfaces::msg::DayTimeInfo>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    enpm663_interfaces::msg::DayTimeInfo day_time_info_msg_;
    std::vector<int> time_of_day_list_;
    std::vector<int> day_of_week_list_;
};

