#include "rclcpp/rclcpp.hpp"
#include "interface_demo_pkg/interface_demo_interface.hpp"
#include <random>
#include <chrono>

void InterfaceDemoInterface::timer_callback()
{
    // Generate random time of day and day of the week
    auto random_time_of_day = time_of_day_list_[rand() % time_of_day_list_.size()];
    auto random_day_of_week = day_of_week_list_[rand() % day_of_week_list_.size()];

    // Set the message fields
    day_time_info_msg_.time_of_day = random_time_of_day;
    day_time_info_msg_.day_of_week = random_day_of_week;
    day_time_info_msg_.time = this->get_clock()->now();

    // Publish the message
    _pub->publish(day_time_info_msg_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceDemoInterface>(
        "interface_demo_cpp");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // This will start the execution
    rclcpp::shutdown();
}