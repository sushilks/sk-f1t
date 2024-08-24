/**
 * Automatic Breaking
 */

#include <aeb/aeb.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aeb::AEB>();

  node->init();

  RCLCPP_INFO(node->get_logger(), "Automatic Emergency Break Controller!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}