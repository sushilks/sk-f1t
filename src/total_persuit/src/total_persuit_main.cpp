// #include <gap_follow/gap_follow.hpp>
/**
 * @file reactive_node.cpp
 * @brief ROS2 node for reactive gap following controller.
 *
 * This file contains the main entry point for the reactive gap following
 * controller. It creates a ROS2 node and initializes the GapFollow class.
 */

#include <total_persuit/total_persuit.hpp>


/**
 * @brief Main entry point for reactive gap following controller
 *
 * This function creates a ROS2 node and initializes the GapFollow class.
 * It then starts the node and waits for it to be shutdown.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return EXIT_SUCCESS if the node is shutdown successfully
 */
int main(int argc, char** argv) {
  // Initialize the ROS2 client library
  rclcpp::init(argc, argv);

  // Create a shared pointer to the GapFollow class
  auto node = std::make_shared<total_persuit::TotalPersuit>();

  // Initialize the node
  node->init();

  // Print a message to the console indicating that the node is running
  RCLCPP_INFO(node->get_logger(), "Total Persuit controller.");

  // Start the node
  rclcpp::spin(node);

  // Shutdown the node
  rclcpp::shutdown();

  // Return successful exit status
  return EXIT_SUCCESS;
}
