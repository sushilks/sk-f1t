#include <debug/debug_line.hpp>

namespace debug {
static uint64_t debugLineId = 0;

/**
 * @brief Constructor for DebugLine
 *
 * @param name The name of the debug line
 * @param node The ROS 2 node that will publish the debug line
 * @param c The color of the debug line
 */
DebugLine::DebugLine(std::string name, rclcpp::Node *node,
                     const std_msgs::msg::ColorRGBA &c) {
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
/**
 * @brief Callback for odom messages.
 *
 * This function is called whenever an odom message is received. It updates the
 * internal state of the debug line with the current position and orientation
 * of the car.
 *
 * @param msg The odom message
 */
void DebugLine::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  car_position_ = msg.get()->pose.pose.position;
  car_orient_ = msg.get()->pose.pose.orientation;
}
/**
 * @brief Publish the debug line with the current timestamp.
 *
 * This function publishes the debug line with the current timestamp and the
 * current position and orientation of the car. If `cleanup_after_send` is true,
 * the debug line is cleared after publishing.
 *
 * @param stamp The timestamp to publish the debug line with
 * @param cleanup_after_send If true, the debug line is cleared after publishing
 */
void DebugLine::send_msg(rclcpp::Time stamp, bool cleanup_after_send) {
  marker_.action = visualization_msgs::msg::Marker::MODIFY;
  marker_.header.stamp = stamp;
  marker_.pose.position = car_position_;
  marker_.pose.orientation = car_orient_;
  marker_pub_->publish(marker_);
  if (cleanup_after_send) msg_init();
}
/**
 * @brief Add a line to the debug line
 *
 * This function adds a line to the debug line by appending two points to the
 * `points` vector of the marker. The points are defined by the two pairs of
 * (x, y) coordinates given as arguments.
 *
 * @param x1 The x-coordinate of the first point of the line
 * @param y1 The y-coordinate of the first point of the line
 * @param x2 The x-coordinate of the second point of the line
 * @param y2 The y-coordinate of the second point of the line
 */
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