#pragma once

/**
 * @file pub_demo_interface.hpp
 * @brief Declares the PubDemoInterface class for publishing string messages at
 * regular intervals.
 *
 * This file defines the PubDemoInterface class, which inherits from rclcpp::Node.
 * It demonstrates creating a simple ROS2 publisher in C++ that periodically
 * publishes messages to a topic.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class PublisherNode
 * @brief A class that publishes string messages at fixed intervals.
 *
 * PublisherNode is a ROS2 node that periodically publishes string messages.
 */
class PubDemoInterface : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a PubDemoInterface with a given name.
   *
   * @param node_name The name of the node.
   * Initializes the publisher, timer, and sets up the timer callback for
   * publishing messages.
   */
  PubDemoInterface(std::string node_name) : Node(node_name)
  {
    leia_topic_msg_ = std_msgs::msg::String();

    // Setup a timer to call timer_callback every 500 milliseconds
    leia_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(500.0)),
        std::bind(&PubDemoInterface::leia_timer_cb, this));

    // Initialize the publisher on topic "leia" with a queue size of 10
    leia_pub_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
  }

private:
  rclcpp::TimerBase::SharedPtr leia_timer_; ///< Timer to trigger publishing.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      leia_pub_; ///< The publisher object.
  std_msgs::msg::String leia_topic_msg_;
  /**
   * @brief Timer callback function that publishes a message.
   *
   * This function constructs a message and publishes the message.
   */
  void leia_timer_cb();
};