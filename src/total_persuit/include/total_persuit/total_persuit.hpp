#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

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
  LOOKAHEAD = 2,

};

class TotalPersuit : public rclcpp::Node {
 public:
  explicit TotalPersuit();
  void init();
  void pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);
  void publishNext();
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void pid_control(double error);
  void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  // void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void process(double x, double y, double yaw, double speed);

  std::shared_ptr<WaypointPublisher> waypoint_publisher_;
  std::shared_ptr<SplinePath> spline_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<DEBUGLINES, std::shared_ptr<debug::DebugLine>> debug_lines_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ack_pub_;

  double kp_, ki_, kd_;
  double prev_error_;
  double error_;
  double integral_;
  double tdist_;
  double max_speed_;
  double la_time_;
  int debug_;
  std::string odom_topic_name_;
  std::string drive_topic_name_;
  std::string scan_topic_name_;
  std::string pose_topic_name_;
  /// Angle ranges for the vehicle
  std::vector<double> angle_ranges_;

  /// Speed ranges for the vehicle
  std::vector<double> speed_ranges_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  double delta_x_ = 0.0;
  double delta_y_ = 0.0;
  double delta_yaw_ = 0.0;
};
}  // namespace total_persuit