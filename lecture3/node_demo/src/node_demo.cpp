#include <rclcpp/rclcpp.hpp>

#include "node_demo/node_demo_interface.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NodeDemoInterface>("node_demo_cpp");

    rclcpp::shutdown();
}