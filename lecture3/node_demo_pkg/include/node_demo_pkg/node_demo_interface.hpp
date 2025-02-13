#pragma once
#include <rclcpp/rclcpp.hpp>

class NodeDemoInterface : public rclcpp::Node {
   public:
    NodeDemoInterface(std::string node_name)
        : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
    }
};