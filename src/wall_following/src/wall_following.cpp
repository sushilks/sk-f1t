
#include <wall_following/wall_following.hpp>

namespace wall_following {

/// Construct a WallFollowing node.
///
/// This constructor creates a ROS node named "wall_following" and initializes
/// the current velocity to 0.0.
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
  speed0_ = declare_parameter("s0", 0.5);
  speed1_ = declare_parameter("s1", 1.0);
  speed2_ = declare_parameter("s2", 1.5);
  speed3_ = declare_parameter("s3", 3.0);
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

  // init debug line
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0;
  c.g = 0.0;
  c.b = 0.0;
  c.a = 0.5;
  debug_lines_[Scan] =
      std::make_shared<debug::DebugLine>("Scan", this, c);  // red scan line
  c.r = 0.0;
  c.b = 1.0;
  debug_lines_[WallDist] =  // blue distance from wall line
      std::make_shared<debug::DebugLine>("WallDist", this, c);
  c.g = 1.0;
  c.b = 0.0;
  debug_lines_[Direction] =  // blue distance from wall line
      std::make_shared<debug::DebugLine>("Direction", this, c);
}
/// \brief rad to degree conversion
///
/// \param r rad value
/// \return degree value
float inline rtod(float r) {  // rad to degree conversion
  return r * 180 / M_PI;
}

/// \brief degree to radian conversion
///
/// \param r rad value
/// \return degree value
float inline dtor(float r) {  //  degree to rad conversion
  return r * M_PI / 180.0;
}

void WallFollowing::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  // RCLCPP_INFO(
  //     get_logger(), "ODOM Msg O[%f %f %f %f] twist A[%f %f %f] L[%f %f
  //     %f]",
  //     // measured Facing Right, Backward,  Left, Forward
  //     msg.get()->pose.pose.orientation.w,  // [R -0.7 B -0.3 L  0.6 F  1.0]
  //     msg.get()->pose.pose.orientation.x,  // [0]
  //     msg.get()->pose.pose.orientation.y,  // [0]
  //     msg.get()->pose.pose.orientation.z,  // [R  0.7 B  1.0 L  0.7 F 0.0]
  //     msg.get()->twist.twist.angular.x, msg.get()->twist.twist.angular.y,
  //     msg.get()->twist.twist.angular.z,  // Turn direction of the vehicle
  //     // movement speed forward with respect to the direction the vehicle
  //     is
  //     // facing likely unit is meter / sec.
  //     msg.get()->twist.twist.linear.x, msg.get()->twist.twist.linear.y,
  //     msg.get()->twist.twist.linear.z);
  current_velocity_ = msg.get()->twist.twist.linear.x;
  for (auto dt : debug_lines_) {
    dt.second->odom_callback(msg);
  }
}

double WallFollowing::get_distance_at_angle(
    const sensor_msgs::msg::LaserScan *dt, double angle) {
  int idx_angle = (angle - dt->angle_min) / dt->angle_increment;
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

alpha = tan -1 [(a cost theta - b) / (a sin theat) ]

 */
double WallFollowing::get_error(const sensor_msgs::msg::LaserScan *dt,
                                double dist, double theta, double lookahead,
                                bool debug) {  // look ahead default 1 meter.
  auto r = get_wall_distance(dt, theta, debug);
  double alpha = r.second;
  double wall_dist = r.first;

  double wall_dist_lookahead =
      wall_dist + lookahead * sin(alpha);  // look ahead distance
  double err = wall_dist_lookahead - dist;
  return err;
}
// double WallFollowing::get_error(
//     const sensor_msgs::msg::LaserScan *dt, double dist, double theta,
//     double lookahead) {  // look ahead default 1 meter.
//   double m = (theta < 0) ? -1.0 : 1.0;

//   double ref_angle = dtor(m * 90);
//   // get 90* sample from both sides
//   // min -> 90 => (90 - min)/increment
//   double dist_90 = get_distance_at_angle(dt, ref_angle);
//   // distance at 0 < theta <= 70
//   // double theta = 90 - 20;
//   double dist_theta = get_distance_at_angle(dt, ref_angle - theta);
//   // alpha = angle of the car to w.r.t the wall
//   double alpha =
//       m * atan((dist_theta * cos(theta) - dist_90) / (dist_theta *
//       sin(theta)));
//   double wall_dist = dist_90 * cos(alpha);  // current distance to wall on
//   right

//   double wall_dist_lookahead =
//       wall_dist + lookahead * sin(alpha);  // look ahead distance
//   double err = wall_dist_lookahead - dist;
//   // RCLCPP_INFO(get_logger(),
//   //             "ERROR[%f] : dist[%2.0f] %f, dist[%2.0f] %f, alpha = %f, "
//   //             "wall_dist %f, lh %f ",
//   //             err, rtod(ref_angle), dist_90, rtod(ref_angle - theta),
//   //             dist_theta, rtod(alpha), wall_dist, wall_dist_lookahead);

//   if (true) {
//     // draw the 90 degree line
//     debug_lines_.at(WallDist)->add_line(0, 0, 0, dist_90 * m);
//     // draw the 90 - theta degree line
//     debug_lines_.at(WallDist)->add_line(0, 0,
//                                         dist_theta * cos(ref_angle - theta),
//                                         dist_theta * sin(ref_angle - theta));
//   }
//   // draw the line to the wall AB
//   debug_lines_.at(WallDist)->add_line(0, 0, -1 * wall_dist * sin(alpha),
//                                       wall_dist * cos(alpha) * m);

//   return err;
// }

std::pair<double, double> WallFollowing::get_wall_distance(
    const sensor_msgs::msg::LaserScan *dt, double theta,
    bool debug) {  // look ahead default 1 meter.
  double m = (theta < 0) ? -1.0 : 1.0;

  double ref_angle = dtor(m * 90);
  // get 90* sample from both sides
  // min -> 90 => (90 - min)/increment
  double dist_90 = get_distance_at_angle(dt, ref_angle);
  // distance at 0 < theta <= 70
  // double theta = 90 - 20;
  double dist_theta = get_distance_at_angle(dt, ref_angle - theta);
  // alpha = angle of the car to w.r.t the wall
  double alpha =
      m * atan((dist_theta * cos(theta) - dist_90) / (dist_theta * sin(theta)));
  double wall_dist = dist_90 * cos(alpha);  // current distance to wall on right

  if (debug) {
    // draw the 90 degree line
    debug_lines_.at(WallDist)->add_line(0, 0, 0, dist_90 * m);
    // draw the 90 - theta degree line
    debug_lines_.at(WallDist)->add_line(0, 0,
                                        dist_theta * cos(ref_angle - theta),
                                        dist_theta * sin(ref_angle - theta));
    // draw the line to the wall AB
    debug_lines_.at(WallDist)->add_line(0, 0, -1 * wall_dist * sin(alpha),
                                        wall_dist * cos(alpha) * m);
  }
  return std::make_pair(wall_dist, alpha);
}

void WallFollowing::pid_control(double error) {
  double angle = 0.0;
  double speed = speed0_;

  // translate the error and velocity to desired
  // speed and angle using a pid controller algo.
  // Steering angle 0 -> 10 degree velocity = 1.5 m/sec
  //                10 -> 20                = 1.0 m/sec
  // default speed = 0.5 m /sec
  angle = kp_ * error +                  // proportional control
          kd_ * (error - prev_error_) +  // derivative control
          ki_ * integral_;               // integral control

  if (angle >= dtor(10) && angle <= dtor(20)) {
    speed = speed1_;  // meter/sec
  } else if (angle < dtor(10) && angle > dtor(5)) {
    speed = speed2_;  // m/s
  } else {
    speed = speed3_;
  }
  // angle = std::max(angle, -0.2);

  if (error < 0.2) {
    integral_ = integral_ + error;
  } else {
    integral_ = 0;
  }
  // RCLCPP_INFO(get_logger(), " Error = %f , Angle = %f", error, angle);
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = speed;
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.steering_angle_velocity = 0.1;
  ack_pub_->publish(drive_msg);
}

void WallFollowing::scan_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // static int j = 10;
  // if (j < 10) {
  //   RCLCPP_INFO(
  //       get_logger(),
  //       "msg: angle:[%f %f %f] range[%f %f] time[%f %f] range:%d ins:%d",
  //       rtod(msg.get()->angle_min), rtod(msg.get()->angle_max),
  //       rtod(msg.get()->angle_increment), (msg.get()->range_min),
  //       (msg.get()->range_max), msg.get()->scan_time,
  //       msg.get()->time_increment, (int)msg.get()->ranges.size(),
  //       (int)msg.get()->intensities.size());
  // }
  prev_error_ = error_;
  double angle = 20;
  auto wall_left = abs(get_wall_distance(msg.get(), dtor(angle)).first);
  auto wall_right = abs(get_wall_distance(msg.get(), dtor(-1.0 * angle)).first);
  for (int j = 10; j < 30; ++j) {
    // wall_left =
    //     std::min(wall_left, abs(get_wall_distance(msg.get(),
    //     dtor(j)).first));
    // wall_right = std::min(
    //     wall_right, abs(get_wall_distance(msg.get(), dtor(-1.0 * j)).first));
  }
  bool tracking_left = true;
  //     wall_right = std::min(wall_right, wall_dist_ * 2);
  double desired_dist = (wall_left + wall_right) / 2;
  // if (((wall_left - wall_dist_) > (wall_right - wall_dist_)) &&
  //     wall_right > wall_dist_) {
  //   tracking_left = false;
  // }
  if (wall_left > 2.0 || wall_right > 2.0) desired_dist = wall_dist_;

  desired_dist = std::min(std::max(desired_dist, 0.6), 2.0);
  // if (wall_left > (wall_dist_ * 2) || wall_right > (wall_dist_ * 2)) {
  // if (abs(wall_left - wall_right) > wall_dist_ * 2) {
  //   desired_dist = wall_dist_;  // std::min(wall_left, wall_right);
  //   //    if (wall_left > wall_right) tracking_left = false;

  //   // if (wall_left > (wall_dist_ * 2)) tracking_left = false;
  // }
  std::string st = "right";
  if (tracking_left) st = "left";
  // RCLCPP_INFO(get_logger(), "Wall Dist [%s] [ %f %f] %f", st.c_str(),
  // wall_left,
  //             wall_right, desired_dist);
  if (true) {
    if (tracking_left) {
      error_ = get_error(msg.get(), desired_dist, dtor(angle), 1.0, true);
    } else {
      error_ =
          get_error(msg.get(), desired_dist, dtor(-1.0 * angle), 1.0, true);
    }
  } else {
    error_ = 0;
    int cnt = 0;
    // double cur_angle = dtor(1);
    for (int j = 10; j < 30; ++j) {
      error_ += get_error(msg.get(), desired_dist, dtor(j), 1.0);
      // cur_angle += msg.get()->angle_increment;5s
      cnt++;
    }
    error_ = error_ / cnt;
  }
  // double e1 = get_error(msg.get(), desired_dist, dtor(10));
  // double e2 = get_error(msg.get(), desired_dist, dtor(15));
  // double e3 = get_error(msg.get(), desired_dist, dtor(20));
  // double e4 = get_error(msg.get(), desired_dist, dtor(25));
  // error_ = (e1 + e2 + e3 + e4) / 4.0;

  if (true) {
    pid_control(error_);
  }

  int cnt = 0;
  double cur_angle = msg.get()->angle_min;
  for (auto dt : msg.get()->ranges) {
    debug_lines_.at(Scan)->add_line(0, 0, dt * cos(cur_angle),
                                    dt * sin(cur_angle));
    cur_angle += msg.get()->angle_increment;
    cnt++;
  }
  debug_lines_.at(Scan)->send_msg(this->get_clock().get()->now());
  debug_lines_.at(WallDist)->send_msg(this->get_clock().get()->now());
  // debug_lines_.at(Direction)->send_msg(this->get_clock().get()->now());
}

}  // namespace wall_following
