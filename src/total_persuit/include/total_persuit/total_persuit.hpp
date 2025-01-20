#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "debug/debug_line.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "spline_path.hpp"
#include "waypoint_publisher.hpp"

namespace total_persuit {

enum DEBUGLINES {
  /**
   * @brief Draw the closest waypoint
   */
  WAYPOINT = 1,
};

class TotalPersuit : public rclcpp::Node {
 public:
  explicit TotalPersuit();
  void init();
  void pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr &pose_msg);
  void publishNext();
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void pid_control(double error);
  void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  std::shared_ptr<WaypointPublisher> waypoint_publisher_;
  std::shared_ptr<SplinePath> spline_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<DEBUGLINES, std::shared_ptr<debug::DebugLine>> debug_lines_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ack_pub_;

  double kp_, ki_, kd_;
  double prev_error_;
  double error_;
  double integral_;
  double tdist_;
  double max_speed_;
  int debug_;
  /// Angle ranges for the vehicle
  std::vector<double> angle_ranges_;

  /// Speed ranges for the vehicle
  std::vector<double> speed_ranges_;
};
}  // namespace total_persuit