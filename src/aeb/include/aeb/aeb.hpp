#ifndef AEB_HPP_
#define AEB_HPP_
#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace aeb {

class AEB : public rclcpp::Node {
 public:
  explicit AEB();
  // ~AEB() = default;
  void init();

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ack_pub_;
  double current_velocity_;
  double full_break_threshold_;
  double p1_break_threshold_;
  double p2_break_threshold_;

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
};

}  // namespace aeb
#endif