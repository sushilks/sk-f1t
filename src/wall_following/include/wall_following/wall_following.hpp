#ifndef WALL_FOLLOWING_HPP_
#define WALL_FOLLOWING_HPP_
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "debug/debug_line.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace wall_following {

enum DebugLineName { Scan = 1, WallDist = 2, Direction = 3 };
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
  //  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  double current_velocity_;
  double kp_, ki_, kd_;
  double speed0_, speed1_, speed2_, speed3_;
  double wall_dist_;
  double servo_offset_;
  double prev_error_;
  double error_;
  double integral_;
  //   visualization_msgs::msg::Marker marker_;
  //   geometry_msgs::msg::Point car_position_;
  //   geometry_msgs::msg::Quaternion car_orient_;
  std::map<DebugLineName, std::shared_ptr<debug::DebugLine>> debug_lines_;

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  double get_error(const sensor_msgs::msg::LaserScan *range_data, double dist,
                   double theta, double lookahead = 1.0, bool debug = false);
  std::pair<double, double> get_wall_distance(
      const sensor_msgs::msg::LaserScan *dt, double theta, bool debug = false);
  void pid_control(double error);
  double get_distance_at_angle(const sensor_msgs::msg::LaserScan *dt,
                               double angle);
};

}  // namespace wall_following
#endif