
#include <lib/common.hpp>
#include <total_persuit/total_persuit.hpp>

using namespace common;

namespace total_persuit {
TotalPersuit::TotalPersuit() : Node("total_persuit") {
  rcl_interfaces::msg::ParameterDescriptor desc_s;
  desc_s.name = "waypoints";
  desc_s.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  desc_s.description = "list of xy coordinates for the waypoint";
  declare_parameter(desc_s.name, std::vector<float>{}, desc_s);

  PARAM_DOUBLE(kp_, "kp", 0.31, "PID controller parameter P")
  PARAM_DOUBLE(ki_, "ki", 0.00, "PID controller parameter I")
  PARAM_DOUBLE(kd_, "kd", 0.71, "PID controller parameter D")
  PARAM_DOUBLE(tdist_, "track_dist", 1.0, "distance to track")
  PARAM_DOUBLE(max_speed_, "max_speed", 10.0, "speed for range 3")
  PARAM_INT(debug_, "debug", 0,
            "bit 0 = enable debug with drawn lines in GVIZ, bit 1 == enable "
            "info printed messages")

  {
    rcl_interfaces::msg::ParameterDescriptor desc_a, desc_s;
    desc_a.name = "angle_range";
    desc_a.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    desc_a.description = "Angle steps and associated speed as an array";
    desc_s.name = "speed_range";
    desc_s.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    desc_s.description = "Angle steps and associated speed as an array";
    declare_parameter(desc_a.name,
                      std::vector<float>{11.0, 5.0, 10.0, 20.0, 40.0}, desc_a);
    declare_parameter(desc_s.name, std::vector<double>{5.0, 3.0, 1.5, 1.0, 0.5},
                      desc_s);
  }
  angle_ranges_ = get_parameter("angle_range").as_double_array();
  speed_ranges_ = get_parameter("speed_range").as_double_array();

  // init debug line - used for debugg lines on the car.
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0;
  c.g = 0.0;
  c.b = 0.0;
  c.a = 0.5;
  debug_lines_[WAYPOINT] =
      std::make_shared<debug::DebugLine>("WAYPOINT", this, c);  // red scan line
}

void TotalPersuit::publishNext() {
  waypoint_publisher_->publishWaypoint();
  spline_path_->publish();

  // RCLCPP_INFO(this->get_logger(), "Published waypoint ");
}

void TotalPersuit::init() {
  // Initialize member variables that depend on shared_from_this()
  waypoint_publisher_ = std::make_shared<WaypointPublisher>(shared_from_this());
  spline_path_ = std::make_shared<SplinePath>(shared_from_this());

  auto waypoints_param = get_parameter("waypoints").as_double_array();
  waypoint_publisher_->loadWaypoints(waypoints_param);
  spline_path_->generate(waypoint_publisher_->getWaypoints());

  // odom sub is needed for orentiation of the car and to draw the debug lines
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10,
      std::bind(&TotalPersuit::odom_callback, this, std::placeholders::_1));

  // Publish to the drive topic
  ack_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

  // Subscribe to the LiDAR topic
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,  // rclcpp::QoS(10),
      std::bind(&TotalPersuit::lidar_callback, this, std::placeholders::_1));

  //
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),  // Publish every 2 seconds
      std::bind(&TotalPersuit::publishNext, this));
}
void TotalPersuit::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  auto pos = msg.get()->pose.pose.position;
  auto orient = msg.get()->pose.pose.orientation;
  // current_velocity_ = msg.get()->twist.twist.linear.x;
  // if (debug_ & 1)
  //   for (auto dt : debug_lines_) {
  //     dt.second->odom_callback(msg);
  //   }
  double x, y, angle;
  double yaw =
      std::atan2(2.0 * (orient.w * orient.z + orient.x * orient.y),
                 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z));

  // RCLCPP_INFO(this->get_logger(), "orient %f %f %f %f [%f]", orient.x,
  // orient.y,
  //             orient.z, orient.w, yaw);
  //  0, -1 -> 0, 0.9 -> 0 -0.99 -> 0 -0.9 (below 0d) -> 0 0.99(abover 0d)
  // tan-1(1) => 45, tan-1(-1) = -45
  //  0.6 0.799 (Facing up ^) -> 0.715 0.698 (up right) -> 0.822 0.56 (up
  //  left)
  // tan-1(0.6/.8)=36.86 tan-1(.8/.6)=53 cos-1() => 41,und sin-1()=>48,und
  //  0.99 -0.04 (<--) 0.99 -0.13(little down)  0.99 0.063(little up)
  //  0.75 -0.66 (down) 0.79 -0.6 (down left)  0.65 -0.755(down right)
  // yaw = -> 0 , -0.2 if 10d lower , 0.2 if 10d hight
  // yaw = 1.692 for up (^) ==> 1.692 rad = 96 deg
  // yaw = -3.13 for back (<--) => -179deg
  // yaw = -1.61 for down (v) => -92 degree
  spline_path_->get_point_on_path(pos.x, pos.y, yaw, tdist_, x, y, angle);
  debug_lines_.at(WAYPOINT)->add_line(pos.x, pos.y, x, y);
  auto err = angle - yaw;
  if (yaw < 0) {
    err = (angle - (dtor(360) + yaw));
  }  // -15 - (360 - 1) =>
  if (err <= dtor(-360)) err += dtor(360);
  if (err >= dtor(360)) err -= dtor(360);
  if (err <= dtor(-180))
    err = dtor(360) + err;
  else if (err >= dtor(180))
    err = dtor(360) - err;
  // err = std::min(err, dtor(360) - err);
  // yaw = 178 ,,, angle = 162
  // yaw = -171 angle = 208
  if (debug_ > 0) {
    RCLCPP_INFO(get_logger(), " yaw = %f , Angle = %f err = %f", rtod(yaw),
                rtod(angle), rtod(err));
  }
  pid_control(err);
  for (auto dt : debug_lines_) {
    dt.second->send_msg(this->get_clock().get()->now());
  }
}
void TotalPersuit::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr &pose_msg) {
  (void)pose_msg;
  // TODO: find the current waypoint to track using methods mentioned in
  // lecture

  // TODO: transform goal point to vehicle frame of reference

  // TODO: calculate curvature/steering angle

  // TODO: publish drive message, don't forget to limit the steering angle.
}
void TotalPersuit::lidar_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  (void)scan_msg;
}
void TotalPersuit::pid_control(double error) {
  double angle = 0.0;

  // translate the error and velocity to desired
  // speed and angle using a pid controller algo.
  // Steering angle 0 -> 10 degree velocity = 1.5 m/sec
  //                10 -> 20                = 1.0 m/sec
  // default speed = 0.5 m /sec
  angle = kp_ * error +                  // proportional control
          kd_ * (error - prev_error_) +  // derivative control
          ki_ * integral_;               // integral control

  // adjust speed based on angle
  double speed = 0.1;
  for (int idx = 0; idx < int(angle_ranges_.size()); ++idx) {
    if (abs(angle) < common::dtor(angle_ranges_.at(idx))) {
      speed = speed_ranges_.at(idx);
      break;
    }
  }

  if (error < 0.2) {
    integral_ = integral_ + error;
  } else {
    integral_ = 0;
  }
  // RCLCPP_INFO(get_logger(), " Error = %f , Angle = %f", error, angle);
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = std::min(max_speed_, speed);
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.steering_angle_velocity = 0.1;
  ack_pub_->publish(drive_msg);
}

}  // namespace total_persuit
