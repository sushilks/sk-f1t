/**
 * Wall Following
 */

#include <wall_following/wall_following.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wall_following::WallFollowing>();

  node->init();

  RCLCPP_INFO(node->get_logger(), "Wall Following Controller.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}