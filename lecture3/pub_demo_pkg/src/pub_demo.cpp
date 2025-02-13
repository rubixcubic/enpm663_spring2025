/**
 * @file publisher_node.cpp
 * @brief Implementation of a ROS 2 publisher node in C++.
 *
 * This file contains the implementation of the PublisherNode class, which is a
 * ROS 2 node designed to publish string messages at regular intervals. It
 * demonstrates a basic ROS 2 publisher node using rclcpp.
 */

#include <rclcpp/rclcpp.hpp>
#include "pub_demo_pkg/pub_demo_interface.hpp"

//=====================================
void PubDemoInterface::leia_timer_cb()
{
  // Set the message
  leia_topic_msg_.data = "Help me Obi-Wan Kenobi, you are my only hope";
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << leia_topic_msg_.data);
  leia_pub_->publish(leia_topic_msg_);
}

/**
 * @brief Main function to initialize and run the PublisherNode.
 *
 * This is the entry point of the program. It initializes the ROS 2 system,
 * creates a PublisherNode, and spins it to continuously publish messages until
 * the program is interrupted or terminated.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code.
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); ///< Initialize ROS 2.
  auto node = std::make_shared<PubDemoInterface>(
      "pub_demo_cpp"); ///< Create an instance of PubDemoInterface.
  rclcpp::spin(node);  ///< Enter a loop, pumping callbacks.
  rclcpp::shutdown();  ///< Shutdown communications and clean up
}