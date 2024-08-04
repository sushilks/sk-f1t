
#include <math.h>

#include <aeb/aeb.hpp>

namespace aeb {

AEB::AEB() : Node("Automatic_Emergency_Breaking", ""), current_velocity_(0.0) {}

void AEB::init() {
  RCLCPP_INFO(this->get_logger(), "Init!");

  p1_break_threshold_ = declare_parameter("p1_break_threshold", 1.5);
  p2_break_threshold_ = declare_parameter("p2_break_threshold", 1.0);
  full_break_threshold_ = declare_parameter("full_break_threshold", 0.8);
  // subascribe to topic
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,  // rclcpp::QoS(10),
      std::bind(&AEB::scan_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10,
      std::bind(&AEB::odom_callback, this, std::placeholders::_1));
  ack_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

  RCLCPP_INFO(get_logger(), "Starting with Thresholds[%f %f %f]",
              p1_break_threshold_, p2_break_threshold_, full_break_threshold_);
}

float rtod(float r) {  // rad to degreen convertion
  return r * 180 / M_PI;
}

void AEB::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
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

void AEB::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
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
  if (j < 10) {
    {
      std::string st;
      for (auto dt : msg.get()->ranges)  // distance in meters
      {
        std::stringstream ss;
        ss << " " << dt;
        st += ss.str();
      }
      RCLCPP_INFO(get_logger(), "--RR-> %s", st.c_str());
    }
    {
      std::string st;
      for (auto dt : msg.get()->intensities) {
        std::stringstream ss;
        ss << " " << dt;
        st += ss.str();
      }
      RCLCPP_INFO(get_logger(), "--II-> %s", st.c_str());
    }
    j++;
  }
  auto angle = msg.get()->angle_min;
  auto &angle_incr = msg.get()->angle_increment;
  auto &range_max = msg.get()->range_max;
  double min_ttc = 0;
  // compute the time to collusion
  for (auto dt : msg.get()->ranges) {
    if (dt < range_max && dt > 0) {
      auto angle_velocity = current_velocity_ * cos(angle);
      double ttc = min_ttc;
      if (current_velocity_ > 0) {  // going forward
        if (angle_velocity > 0) {
          ttc = dt / angle_velocity;
        }
      } else if (angle_velocity != 0) {  // going backward
        // reverser break is not working -- car keeps moving backward
        // with momentum
        // adjust for the size of the car
        auto ttc = (dt - 0.2) / angle_velocity;
        if (ttc > 0 && (min_ttc > ttc || min_ttc == 0)) {
          min_ttc = ttc;
        }
      }
      if (ttc > 0 && (min_ttc > ttc || min_ttc == 0)) {
        min_ttc = ttc;
      }
    }
    angle += angle_incr;
  }
  if (min_ttc > 0) {
    if (min_ttc < 0.8) {
      RCLCPP_INFO(get_logger(), "MIN TTC BRK => %f", min_ttc);
      ackermann_msgs::msg::AckermannDriveStamped m;
      m.drive.speed = 0;
      m.drive.acceleration = 0;
      m.drive.steering_angle = 0;
      m.drive.steering_angle_velocity = 0;
      ack_pub_->publish(m);
    } else {
      //  RCLCPP_INFO(get_logger(), "MIN TTC    => %f", min_ttc);
    }
  }
}

}  // namespace aeb