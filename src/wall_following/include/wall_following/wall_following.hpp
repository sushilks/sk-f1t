#ifndef WALL_FOLLOWING_HPP_
#define WALL_FOLLOWING_HPP_
#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace wall_following {

class WallFollowing : public rclcpp::Node {
 public:
  explicit WallFollowing();
  // ~AEB() = default;
  void init();

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ack_pub_;
  double current_velocity_;
  double kp_, ki_, kd_;
  double wall_dist_;
  double servo_offset_;
  double prev_error_;
  double error_;
  double integral_;

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  double get_error(const sensor_msgs::msg::LaserScan *range_data, double dist,
                   double theta, double lookahead = 1.0);
  void pid_control(double error);
  double get_distance_at_angle(const sensor_msgs::msg::LaserScan *dt,
                               double angle);
};

}  // namespace wall_following
#endif