#include <debug/debug_line.hpp>

namespace debug {
static uint64_t debugLineId = 0;

DebugLine::DebugLine(std::string name, rclcpp::Node *node,
                     std_msgs::msg::ColorRGBA &c) {
  marker_.header.frame_id = "map";
  //  marker_.header.stamp = node->get_clock().get()->now();
  marker_.id = ++debugLineId;
  marker_.ns = "debug";
  marker_.text = "debug lines";
  marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.pose.position.x = 0.0;
  marker_.pose.position.y = 0.0;
  marker_.pose.position.z = 0.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = 0.01;
  marker_.scale.y = 0.01;
  marker_.scale.z = 0.01;
  marker_.color = c;
  name_ = name;
  marker_pub_ =
      node->create_publisher<visualization_msgs::msg::Marker>(name.c_str(), 10);
}
void DebugLine::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  car_position_ = msg.get()->pose.pose.position;
  car_orient_ = msg.get()->pose.pose.orientation;
}
void DebugLine::send_msg(rclcpp::Time stamp, bool cleanup_after_send) {
  marker_.action = visualization_msgs::msg::Marker::MODIFY;
  marker_.header.stamp = stamp;
  marker_.pose.position = car_position_;
  marker_.pose.orientation = car_orient_;
  marker_pub_->publish(marker_);
  if (cleanup_after_send) msg_init();
}
void DebugLine::add_line(double x1, double y1, double x2, double y2) {
  geometry_msgs::msg::Point p;
  p.x = x1;
  p.y = y1;
  p.z = 0;
  marker_.points.push_back(p);
  p.x = x2;
  p.y = y2;
  marker_.points.push_back(p);
}

}  // namespace debug