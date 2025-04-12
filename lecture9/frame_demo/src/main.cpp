#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "frame_demo/broadcaster_demo.hpp"
#include "frame_demo/kdl_frame_demo.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create nodes
    auto kdl_frame_demo_node = std::make_shared<frame_demo::KDLFrameDemo>();
    auto broadcaster_demo_node = std::make_shared<frame_demo::BroadcasterDemo>();

    // Set up multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(kdl_frame_demo_node);
    executor.add_node(broadcaster_demo_node);

    // Spin executor
    executor.spin();

    rclcpp::shutdown();
}