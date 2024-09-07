#include <debug/debug_line.hpp>

namespace debug {
/**
 * @brief DebugLine class for publishing debug lines using the visualization_msgs::msg::Marker message.
 *
 * This class is used to publish debug lines using the visualization_msgs::msg::Marker message.
 * A debug line is a line that is drawn on the map to help with debugging.
 */
class DebugLine {
public:
  /**
   * @brief Constructor for DebugLine.
   *
   * @param name The name of the debug line.
   * @param node The node that is publishing the debug line.
   * @param c The color of the debug line.
   */
  DebugLine(std::string name, rclcpp::Node *node,
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

  /**
   * @brief Odom callback for updating the debug line with the current car position and orientation.
   *
   * @param msg The odometry message.
   */
  void odom_callback(
      const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    car_position_ = msg.get()->pose.pose.position;
    car_orient_ = msg.get()->pose.pose.orientation;
  }

  /**
   * @brief Send the debug line message.
   *
   * @param stamp The timestamp of the message.
   * @param cleanup_after_send Whether to clean up the debug line after sending it.
   */
  void send_msg(rclcpp::Time stamp, bool cleanup_after_send) {
    marker_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_.header.stamp = stamp;
    marker_.pose.position = car_position_;
    marker_.pose.orientation = car_orient_;
    marker_pub_->publish(marker_);
    if (cleanup_after_send) msg_init();
  }

  /**
   * @brief Add a line segment to the debug line.
   *
   * @param x1 The x-coordinate of the start point of the line segment.
   * @param y1 The y-coordinate of the start point of the line segment.
   * @param x2 The x-coordinate of the end point of the line segment.
   * @param y2 The y-coordinate of the end point of the line segment.
   */
  void add_line(double x1, double y1, double x2, double y2) {
    geometry_msgs::msg::Point p;
    p.x = x1;
    p.y = y1;
    p.z = 0;
    marker_.points.push_back(p);
    p.x = x2;
    p.y = y2;
    marker_.points.push_back(p);
  }

private:
  /**
   * @brief Initialize the debug line message.
   */
  void msg_init() { marker_.points.clear(); }

  // A counter for the debug line ID. This is used to keep track of the
  // debug lines that are being published.
  static uint64_t debugLineId = 0;

  // The debug line message.
  visualization_msgs::msg::Marker marker_;

  // The name of the debug line.
  std::string name_;

  // The publisher of the debug line.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // The current position of the car.
  geometry_msgs::msg::Point car_position_;

  // The current orientation of the car.
  geometry_msgs::msg::Quaternion car_orient_;
};

}  // namespace debug
