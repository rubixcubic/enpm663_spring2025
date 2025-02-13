#pragma once
#include <rclcpp/rclcpp.hpp>

/**
 * @class NodeDemoInterface
 * @brief A simple ROS 2 node demonstration class.
 *
 * This class creates a ROS 2 node that logs a greeting message upon initialization.
 * It is designed to be a minimal example of a ROS 2 node using `rclcpp::Node`.
 */
class NodeDemoInterface : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for the NodeDemoInterface class.
     *
     * Initializes the node with the specified name and logs a message upon creation.
     *
     * @param node_name The name of the ROS 2 node.
     */
    NodeDemoInterface(std::string node_name)
        : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
    }
};
