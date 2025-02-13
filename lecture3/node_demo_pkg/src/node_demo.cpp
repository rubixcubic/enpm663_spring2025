/**
 * @file main.cpp
 * @brief Entry point for the ROS 2 node using NodeDemoInterface.
 *
 * This file initializes the ROS 2 system, creates a shared instance of the 
 * `NodeDemoInterface` node, and manages the ROS 2 lifecycle.
 */

#include <rclcpp/rclcpp.hpp>
#include "node_demo_pkg/node_demo_interface.hpp"

/**
 * @brief Main function for the ROS 2 node.
 *
 * This function initializes ROS 2, creates an instance of `NodeDemoInterface`, 
 * and prepares the node for execution. The `rclcpp::spin(node);` line is commented 
 * out, meaning the node is currently not being actively spun.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NodeDemoInterface>("node_demo_cpp");
    // rclcpp::spin(node);  ///< Uncomment to keep the node running.

    rclcpp::shutdown();
    return 0;
}
