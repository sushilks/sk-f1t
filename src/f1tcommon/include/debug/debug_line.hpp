#pragma once
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "nav_msgs/msg/odometry.hpp"

namespace debug {
/**
 * @brief Class for visualizing debug lines in RViz
 *
 * This class is used to publish debug lines in RViz. It is used to visualize
 * the path of the car in the map.
 */
class DebugLine {
 public:
  /**
   * @brief Constructor for DebugLine
   *
   * @param name The name of the debug line
   * @param node The ROS 2 node that will publish the debug line
   * @param c The color of the debug line
   */
  explicit DebugLine(std::string name, rclcpp::Node *node,
                     const std_msgs::msg::ColorRGBA &c);

  /**
   * @brief Callback function for odom messages
   *
   * @param msg The odom message
   */
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * @brief Add a line to the debug line
   *
   * @param x1 The x-coordinate of the first point of the line
   * @param y1 The y-coordinate of the first point of the line
   * @param x2 The x-coordinate of the second point of the line
   * @param y2 The y-coordinate of the second point of the line
   */
  void add_line(double x1, double y1, double x2, double y2);

  /**
   * @brief Publish the debug line
   *
   * @param stamp The time stamp of the message
   * @param cleanup_after_send If true, the debug line will be cleared after
   * publishing
   */
  void send_msg(rclcpp::Time stamp, bool cleanup_after_send = true);

  /**
   * @brief Clear the debug line
   */
  void msg_init() { marker_.points.clear(); }

 private:
  /**
   * @brief The debug line message
   */
  visualization_msgs::msg::Marker marker_;

  /**
   * @brief The position of the car
   */
  geometry_msgs::msg::Point car_position_;

  /**
   * @brief The orientation of the car
   */
  geometry_msgs::msg::Quaternion car_orient_;

  /**
   * @brief The name of the debug line
   */
  std::string name_;

  /**
   * @brief The publisher of the debug line
   */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
}  // namespace debug
