#include <rclcpp/rclcpp.hpp>
#include "executor_demo/executor_demo_interface.hpp"

/**
 * @brief Main function to create the AVActionsInterface node
 *
 * @param argc  number of command line arguments
 * @param argv  command line arguments
 * @return int  0 if successful
 */
// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<SingleThreadedExecutorInterface>(
//       "single_threaded_executor_demo");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
// }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SingleThreadedExecutorInterface>(
      "single_threaded_executor_demo");

  // create an executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // add more nodes if needed
  // executor.add_node(node2);
  // executor.add_node(node3);
  
  executor.spin(); // This will start the execution
 
  rclcpp::shutdown();
}