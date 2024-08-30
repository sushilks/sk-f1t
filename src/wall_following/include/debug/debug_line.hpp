#ifndef LINE_DEBUG_HPP_
#define LINE_DEBUG_HPP_
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "nav_msgs/msg/odometry.hpp"

namespace debug {
static uint64_t debugLineId = 0;
class DebugLine {
 public:
  explicit DebugLine(std::string name, rclcpp::Node *node,
                     std_msgs::msg::ColorRGBA &c);

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  //  void send_msg();
  void add_line(double x1, double y1, double x2, double y2);
  void send_msg(rclcpp::Time stamp, bool cleanup_after_send = true);
  void msg_init() { marker_.points.clear(); }

 private:
  visualization_msgs::msg::Marker marker_;
  geometry_msgs::msg::Point car_position_;
  geometry_msgs::msg::Quaternion car_orient_;
  std::string name_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
}  // namespace debug
#endif