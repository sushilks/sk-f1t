
#include <wall_following/wall_following.hpp>

namespace wall_following {

WallFollowing::WallFollowing()
    : Node("wall_following", ""), current_velocity_(0.0) {}

/// Initialize the WallFollowing node.
///
/// Subscribes to the /scan and /ego_racecar/odom topics and advertises on the
/// /drive topic.
///
void WallFollowing::init() {
  RCLCPP_INFO(this->get_logger(), "Init!");
  kp_ = declare_parameter("kp", 0.51);
  kd_ = declare_parameter("kd", 0.71);
  ki_ = declare_parameter("ki", 0.0);
  wall_dist_ = declare_parameter("wall_dist", 0.51);
  // subascribe to topic
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,  // rclcpp::QoS(10),
      std::bind(&WallFollowing::scan_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10,
      std::bind(&WallFollowing::odom_callback, this, std::placeholders::_1));
  ack_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

  RCLCPP_INFO(get_logger(), "Starting with PID [%f %f %f] wall_dist %f", kp_,
              ki_, kd_, wall_dist_);
}
float inline rtod(float r) {  // rad to degreen convertion
  return r * 180 / M_PI;
}
float inline dtor(float r) {  // rad to degreen convertion
  return r * M_PI / 180.0;
}

void WallFollowing::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  // RCLCPP_INFO(
  //     get_logger(), "ODOM Msg O[%f %f %f %f] twist A[%f %f %f] L[%f %f %f]",
  //     // measured Facing Right, Backward,  Left, Forward
  //     msg.get()->pose.pose.orientation.w,  // [R -0.7 B -0.3 L  0.6 F  1.0]
  //     msg.get()->pose.pose.orientation.x,  // [0]
  //     msg.get()->pose.pose.orientation.y,  // [0]
  //     msg.get()->pose.pose.orientation.z,  // [R  0.7 B  1.0 L  0.7 F 0.0]
  //     msg.get()->twist.twist.angular.x, msg.get()->twist.twist.angular.y,
  //     msg.get()->twist.twist.angular.z,  // Turn direction of the vehicle
  //     // movement speed forward with respect to the direction the vehicle is
  //     // facing likely unit is meter / sec.
  //     msg.get()->twist.twist.linear.x, msg.get()->twist.twist.linear.y,
  //     msg.get()->twist.twist.linear.z);
  current_velocity_ = msg.get()->twist.twist.linear.x;
}
double WallFollowing::get_distance_at_angle(
    const sensor_msgs::msg::LaserScan *dt, double angle) {
  int idx_angle = (angle - rtod(dt->angle_min)) / rtod(dt->angle_increment);
  // RCLCPP_INFO(get_logger(), "dt.range %d idx %d,  min = %f increment = %f",
  //             (int)dt->ranges.size(), idx_angle, rtod(dt->angle_min),
  //             rtod(dt->angle_increment));
  return dt->ranges.at(idx_angle);
}

/**
 * @brief Calculate the error between the current distance to wall and the
 * set point
 * @param dt: The laser scan message
 * @param dist: The set point distance
 * @param lookahead: The look ahead distance in meter, default is 1 meter
 * @return The error between the current distance to wall and the set point
 *
 * The look ahead distance is used to calculate the distance to the wall
 * ahead of the vehicle. The error is calculated as the difference between
 * the current distance to wall and the set point.
 */
double WallFollowing::get_error(
    const sensor_msgs::msg::LaserScan *dt, double dist, double theta,
    double lookahead) {  // look ahead default 1 meter.

  // get 90* sample from both sides
  // min -> 90 => (90 - min)/increment
  double dist_90 = get_distance_at_angle(dt, 90);
  // distance at 0 < theta <= 70
  // double theta = 90 - 20;
  double dist_theta = get_distance_at_angle(dt, theta);

  double theta_r = dtor(theta);
  // alpha = angle of the car to w.r.t the wall
  double alpha =
      atan((dist_theta * cos(theta_r) - dist_90) / (dist_theta * sin(theta_r)));
  double wall_dist = dist_90 * cos(alpha);  // current distance to wall on right
  double wall_dist_lookahead =
      wall_dist + lookahead * sin(alpha);  // look ahead distance
  double err = wall_dist_lookahead - dist;
  RCLCPP_INFO(get_logger(),
              "ERROR[%f] : dist_90 %f, dist[%f] %f, alpha = %f, wall_dist %f, "
              "lh %f ",
              err, dist_90, theta, dist_theta, alpha, wall_dist,
              wall_dist_lookahead);
  return err;
}

void WallFollowing::pid_control(double error) {
  double angle = 0.0;
  double speed = 0.5;

  // translate the error and velocity to desired
  // speed and angle using a pid controller algo.
  // Steering angle 0 -> 10 degree velocity = 1.5 m/sec
  //                10 -> 20                = 1.0 m/sec
  // default speed = 0.5 m /sec
  angle = kp_ * error +                  // proportional control
          kd_ * (error - prev_error_) +  // derivative control
          ki_ * integral_;               // integral control

  if (angle >= dtor(10) && angle <= dtor(20)) {
    speed = 1.0;  // meter/sec
  } else if (angle < dtor(10)) {
    speed = 1.5;  // m/s
  }

  if (error < 0.1) {
    integral_ = integral_ + error;
  } else {
    integral_ = 0;
  }
  // RCLCPP_INFO(get_logger(), " Error = %f , Angle = %f", error, angle);
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = speed;
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.steering_angle_velocity = 1.0;
  ack_pub_->publish(drive_msg);
}

void WallFollowing::scan_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  static int j = 10;
  if (j < 10) {
    RCLCPP_INFO(
        get_logger(),
        "msg: angle:[%f %f %f] range[%f %f] time[%f %f] range:%d ins:%d",
        rtod(msg.get()->angle_min), rtod(msg.get()->angle_max),
        rtod(msg.get()->angle_increment), (msg.get()->range_min),
        (msg.get()->range_max), msg.get()->scan_time, msg.get()->time_increment,
        (int)msg.get()->ranges.size(), (int)msg.get()->intensities.size());
  }
  // auto angle = msg.get()->angle_min;
  // auto &angle_incr = msg.get()->angle_increment;
  // auto &range_max = msg.get()->range_max;
  // double min_ttc = 0;

  //
  prev_error_ = error_;
  // error_ = get_error(msg.get(), wall_dist_, 70);
  double e1 = get_error(msg.get(), wall_dist_, 70);
  double e2 = get_error(msg.get(), wall_dist_, 65);
  double e3 = get_error(msg.get(), wall_dist_, 60);
  double e4 = get_error(msg.get(), wall_dist_, 55);
  // RCLCPP_INFO(get_logger(), "Error Spread [ %f %f %f %f]", e1, e2, e3, e4);
  error_ = (e1 + e2 + e3 + e4) / 4.0;
  pid_control(error_);
  //   // compute the time to collusion
  //   for (auto dt : msg.get()->ranges) {
  //     if (dt < range_max && dt > 0) {
  //       auto angle_velocity = current_velocity_ * cos(angle);
  //       double ttc = min_ttc;
  //       if (current_velocity_ > 0) {  // going forward
  //         if (angle_velocity > 0) {
  //           ttc = dt / angle_velocity;
  //         }
  //       } else if (angle_velocity != 0) {  // going backward
  //         // reverser break is not working -- car keeps moving backward
  //         // with momentum
  //         // adjust for the size of the car
  //         auto ttc = (dt - 0.2) / angle_velocity;
  //         if (ttc > 0 && (min_ttc > ttc || min_ttc == 0)) {
  //           min_ttc = ttc;
  //         }
  //       }
  //       if (ttc > 0 && (min_ttc > ttc || min_ttc == 0)) {
  //         min_ttc = ttc;
  //       }
  //     }
  //     angle += angle_incr;
  //   }
  //   if (min_ttc > 0) {
  //     if (min_ttc < 0.8) {
  //       RCLCPP_INFO(get_logger(), "MIN TTC BRK => %f", min_ttc);
  //       ackermann_msgs::msg::AckermannDriveStamped m;
  //       m.drive.speed = 0;
  //       m.drive.acceleration = 0;
  //       m.drive.steering_angle = 0;
  //       m.drive.steering_angle_velocity = 0;
  //       ack_pub_->publish(m);
  //     } else {
  //       //  RCLCPP_INFO(get_logger(), "MIN TTC    => %f", min_ttc);
  //     }
  // }
}

}  // namespace wall_following