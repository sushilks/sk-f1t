#ifndef SPLINE_PATH_HPP
#define SPLINE_PATH_HPP

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace total_persuit {

class SplinePath {
 public:
  SplinePath(rclcpp::Node::SharedPtr node);

  void generate(const std::vector<std::array<double, 2>>& waypoints);
  void publish();
  void get_point_on_path(double in_x, double in_y, double yaw, double dist,
                         double& out_x, double& out_y, double& out_angle);

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::array<double, 2>> waypoints_;
  std::vector<std::array<double, 2>> interpolated_points_;
  std::vector<std::array<double, 2>> generateSpline(
      const std::vector<std::array<double, 2>>& waypoints);
};

}  // namespace total_persuit

#endif  // SPLINE_PATH_HPP
