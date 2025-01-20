
#include <gap_follow/gap_follow.hpp>
#include <lib/common.hpp>

namespace gap_follow {

GapFollow::GapFollow() : Node("gap_follow") {}

/**
 * @brief Constructor for GapFollow class
 *
 * This function does the following:
 * 1. Initializes the logger
 * 2. Declares the parameters for PID control
 * 3. Subscribes to the LiDAR topic
 * 4. Publishes to the drive topic
 */
void GapFollow::init() {
  RCLCPP_INFO(this->get_logger(), "Init!");
  // Declare parameters for PID control

  PARAM_DOUBLE(max_speed_, "max_speed", 10.0,
               "maximum speed allowed for the vehicle");

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

  PARAM_INT(range_average_window_size_, "range_average_window_size", 1,
            "average samples in a window to smoothen the range data")

  PARAM_DOUBLE(drivable_angle_, "drivable_angle", 90.0,
               "max angle that is valid in the range data as drivable target, "
               "it's on both side of the car so +- 90 degrees")

  PARAM_DOUBLE(max_dist_threshold_, "max_dist_threshold", 10.0,
               "dont consider distances higher than this as part of the "
               "solution")

  PARAM_DOUBLE(min_dist_threshold_, "min_dist_threshold", 0.01,
               "dont consider distances less than this as part of the "
               "solution")

  PARAM_DOUBLE(car_radius_, "car_radius", 0.3,
               "Car size in radius in meters, this will be used to expand the "
               "obstructions in the track")

  PARAM_DOUBLE(
      disparity_threshold_, "disparity_threshold", 0.5,
      "Distance changes higher than this will be considered as obstacles that "
      "will have car_radius applied to it to avoid collusion")

  PARAM_DOUBLE(
      edge_distance_threshold_, "edge_distance_threshold", 0.2,
      "Distance below this will be considered too close and the car will try "
      "to maintain neutral heading when some thing gets this close")

  PARAM_INT(debug_, "debug", 2,
            "bit 0 = enable debug with drawn lines in GVIZ, bit 1 == enable "
            "info printed messages")

  // Subscribe to the LiDAR topic
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidarscan_topic, 10,  // rclcpp::QoS(10),
      std::bind(&GapFollow::lidar_callback, this, std::placeholders::_1));

  // Publish to the drive topic
  ack_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, 10);

  // odom sub is needed for orentiation of the car and to draw the debug lines
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ego_racecar/odom", 10,
      std::bind(&GapFollow::odom_callback, this, std::placeholders::_1));

  // init debug line - used for debugg lines on the car.
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0;
  c.g = 0.0;
  c.b = 0.0;
  c.a = 0.5;
  debug_lines_[GAPS] =
      std::make_shared<debug::DebugLine>("GAPS", this, c);  // red scan line
  c.r = 0.0;
  c.b = 1.0;
  debug_lines_[SHORTEST] =  // blue distance from wall line
      std::make_shared<debug::DebugLine>("SHORTEST", this, c);
  c.g = 1.0;
  c.b = 0.0;
  debug_lines_[DRIVE] =  // blue distance from wall line
      std::make_shared<debug::DebugLine>("DRIVE", this, c);
  c.g = 1.0;
  c.b = 1.0;
  debug_lines_[SCAN] =  // blue distance from wall line
      std::make_shared<debug::DebugLine>("SCAN", this, c);

  // Print the parameters to the console
  RCLCPP_INFO(get_logger(), "Starting with gap follow controller");
}

void GapFollow::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  current_velocity_ = msg.get()->twist.twist.linear.x;
  if (debug_ & 1)
    for (auto dt : debug_lines_) {
      dt.second->odom_callback(msg);
    }
}
// returns the min distance point
float GapFollow::preprocess_lidar(const std::vector<float>& ranges,
                                  std::vector<float>& rdata,
                                  std::vector<int>& closest_points,
                                  int drivable_angle_idx,
                                  float angle_increment) {
  // Preprocess the LiDAR scan array. Expert implementation includes:
  // 1.Setting each value to the mean over some window
  // 2.Rejecting high values (eg. > 3m)
  float avg = 0.0;
  std::vector<float> win_data;
  float min_dist;
  int min_dist_idx = -1;
  win_data.resize(range_average_window_size_);
  rdata.resize(ranges.size());
  closest_points.resize(0);
  std::map<int, std::pair<float, bool>> disparity_idx;
  for (int rIdx = 0; rIdx < int(ranges.size()); rIdx++) {
    auto rr = ranges.at(rIdx);
    float r = std::min(rr, max_dist_threshold_);
    avg += r;
    if (int(rIdx) >= range_average_window_size_) {
      avg -= win_data[rIdx % range_average_window_size_];
      rdata[rIdx] = avg / range_average_window_size_;
    } else {
      rdata[rIdx] = r;
    }
    win_data[rIdx % range_average_window_size_] = r;

    if (rIdx != 0 &&
        abs(rdata[rIdx] - rdata[rIdx - 1]) > disparity_threshold_) {
      disparity_idx[rIdx] =
          std::make_pair(std::min(rdata[rIdx], rdata[rIdx - 1]),
                         rdata[rIdx] > rdata[rIdx - 1]);
    }

    if (int(rIdx) >= drivable_angle_idx &&
        int(rIdx) <= (int(ranges.size()) - drivable_angle_idx)) {
      if (min_dist_idx == -1 || min_dist > rdata[rIdx]) {
        min_dist_idx = rIdx;
        min_dist = rdata[rIdx];
        closest_points.clear();
        closest_points.push_back(rIdx);
      } else if (min_dist == rdata[rIdx]) {
        closest_points.push_back(rIdx);
      }
    }
  }

  // expand the disparity
  for (auto dt : disparity_idx) {
    int idx = dt.first;
    float dist = dt.second.first;  // (step_up) ? idx - 1 : idx
    bool step_up = dt.second.second;

    float angle = atan(car_radius_ / dist);
    int expand_idx = int(angle / angle_increment);

    for (int i = 0; i < expand_idx; ++i) {
      if (step_up) {
        if (idx + i < int(rdata.size())) {
          rdata[idx + i] = std::min(rdata[idx + i], dist);
        }
      } else {
        if (idx - i > 0) {
          rdata[idx - i] = std::min(rdata[idx - i], dist);
        }
      }
    }
  }

  return min_dist;
}

/**
 * @brief Find the max gap in the free space ranges
 *
 * This function will find the max gap in the free space ranges by iterating
 * over the ranges and find the longest sequence of points that are drivable.
 *
 * @param ranges the free space ranges
 * @param gap the start and end index of the max gap
 * @param drivable_angle_idx the index of the drivable angle
 */
void GapFollow::find_max_gap(const std::vector<float>& ranges,
                             std::pair<int, int>& gap, int drivable_angle_idx) {
  // Return the start index & end index of the max gap in free_space_ranges
  if (ranges.empty()) {
    throw std::invalid_argument("ranges is empty");
  }
  if (drivable_angle_idx > int(ranges.size() / 2)) {
    throw std::invalid_argument("drivable_angle_idx is out of range");
  }
  auto active = false;
  int start = 0;
  int end = 0;
  gap.first = gap.second = 0;
  int last = int(ranges.size()) - drivable_angle_idx - 1;
  for (int idx = drivable_angle_idx; idx <= last; ++idx) {
    if (ranges.at(idx) < min_dist_threshold_ || idx == last) {
      if (active) {
        end = idx;
        active = false;
        if ((end - start) > (gap.second - gap.first)) {
          gap.first = start;
          gap.second = end;
        }
      }
      start = idx;
      end = idx;
    } else if (!active) {
      active = true;
      start = idx;
      end = idx;
    }
  }
}

/**
 * @brief Find the best point in the max gap
 *
 * @param rdata the preprocessed LiDAR scan array
 * @param gap the start and end index of the max gap
 * @return the index of the best point in the gap
 *
 * This function will find the best point in the max gap by iterating over
 * the gap and find the longest sequence of points that are drivable.
 */
int GapFollow::find_best_point(std::vector<float>& rdata,
                               std::pair<int, int>& gap) {
  // Start_i & end_i are start and end indicies of max-gap range, respectively
  // Return index of best point in ranges
  // Naive: Choose the furthest point within ranges and go there
  if (rdata.empty()) {
    throw std::invalid_argument("rdata is empty");
  }
  if (gap.first >= int(rdata.size()) || gap.second >= int(rdata.size())) {
    throw std::invalid_argument("gap indices are out of range");
  }
  // float best_idx = 0;
  float best_dist = 0;
  for (int i = gap.first; i < gap.second; i++) {
    if (rdata[i] > best_dist) {
      best_dist = rdata[i];
      // best_idx = i;
    }
  }
  float tollerance = 0.1;
  bool active = false;
  int start = 0;
  int end = 0;
  int bstart = 0;
  int bend = 0;
  for (int i = gap.first; i < gap.second; i++) {
    if (abs(rdata[i] - best_dist) < tollerance && i != (gap.second - 1)) {
      if (!active) {
        active = true;
        start = i;
      }
      end = i;
    } else if (active || i == (gap.second - 1)) {
      if (!active) {
        start = i;
      }
      end = i;
      active = false;
      if ((bend - bstart) <= (end - start)) {
        bend = end;
        bstart = start;
      }
    }
  }

  return (bstart + bend) >> 1;
}

/**
 * @brief Check how close we are to the wall
 *
 * @param dt a pointer to the most recent LiDAR scan
 * @return a pair of floats (left, right) representing the closest distance
 * to the wall on the left and right sides of the car
 */
std::pair<float, float> GapFollow::check_wall_closeness(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr dt) {
  float angle_st = common::dtor(90);
  float angle_ed = common::dtor(120);
  auto msg = dt.get();

  float left = 100.0;
  float right = 100.0;
  // left side
  for (int idx = int((0 - angle_ed - msg->angle_min) / msg->angle_increment);
       idx < int((0 - angle_st - msg->angle_min) / msg->angle_increment);
       idx++) {
    left = std::min(left, msg->ranges[idx]);
  }
  // right size
  for (int idx = int((angle_st - msg->angle_min) / msg->angle_increment);
       idx < int((angle_ed - msg->angle_min) / msg->angle_increment); idx++) {
    right = std::min(right, msg->ranges[idx]);
  }
  return std::make_pair(left, right);
}

/**
 * @brief Callback for the LiDAR sensor.
 *
 * 1. Preprocess the LiDAR data by rejecting high values and calculating the
 * mean over some window.
 * 2. Find the closest point to the LiDAR in the range and zero out all the
 * location expanding up to the car radius.
 * 3. Find the max length gap in the LiDAR data.
 * 4. Find the best point in the gap.
 * 5. Publish a Drive message with the best angle and speed.
 * 6. If debug is enabled, draw the shortest angle, the gap cone, the
 * direction angle and the full scan result.
 */
void GapFollow::lidar_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  auto angle_range = common::dtor(drivable_angle_);  // +-  degree
  // angle_idx is the starting index where we start considering the range data
  int angle_idx = int(angle_range / scan_msg.get()->angle_increment);
  angle_idx = int(scan_msg.get()->ranges.size() / 2) - angle_idx;

  std::vector<float> rdata;
  std::vector<int> min_dist_idx;
  // preprocess the lidar data and populate the rdata vector.
  float min_dist = preprocess_lidar(scan_msg.get()->ranges, rdata, min_dist_idx,
                                    angle_idx, scan_msg.get()->angle_increment);

  if (debug_ & 1) {  // draw the shortest angle
    for (auto best_idx : min_dist_idx) {
      float cangle = scan_msg.get()->angle_min +
                     scan_msg.get()->angle_increment * best_idx;
      debug_lines_.at(SHORTEST)->add_line(0, 0, rdata[best_idx] * cos(cangle),
                                          rdata[best_idx] * sin(cangle));
    }
  }

  // Find closest point to LiDAR in the range
  // and zero out all the location expanding up to car radius
  int zero_radius = 0;
  for (int idx : min_dist_idx) {
    float dist = rdata[idx];
    float angle = atan(car_radius_ / dist);
    zero_radius = int(angle / scan_msg.get()->angle_increment);

    int start = std::max(idx - zero_radius, 0);
    int end = std::min(idx + zero_radius, int(rdata.size()) - 1);
    for (int i = start; i <= end; i++) {
      rdata[i] = 0;
    }
  }

  if (debug_ & 1) {
    if (true) {  // draw the SCAN cone.
      float cangle = scan_msg.get()->angle_min +
                     scan_msg.get()->angle_increment * angle_idx;
      for (int i = angle_idx;
           i < (int(scan_msg.get()->ranges.size()) - angle_idx); ++i) {
        debug_lines_.at(SCAN)->add_line(0, 0, rdata[i] * cos(cangle),
                                        rdata[i] * sin(cangle));
        cangle += scan_msg.get()->angle_increment;
      }
    } else {  // draw the full scan result
      float cangle = scan_msg.get()->angle_min;
      for (int i = 0; i < int(scan_msg.get()->ranges.size()); ++i) {
        debug_lines_.at(SCAN)->add_line(
            0, 0, scan_msg.get()->ranges[i] * cos(cangle),
            scan_msg.get()->ranges[i] * sin(cangle));
        cangle += scan_msg.get()->angle_increment;
      }
    }
  }

  std::pair<int, int> gap;
  find_max_gap(rdata, gap, angle_idx);

  if (debug_ & 1) {  // draw the gap cone.
    float cangle =
        scan_msg.get()->angle_min + scan_msg.get()->angle_increment * gap.first;
    for (int i = gap.first; i < gap.second; ++i) {
      debug_lines_.at(GAPS)->add_line(0, 0, rdata[i] * cos(cangle),
                                      rdata[i] * sin(cangle));
      cangle += scan_msg.get()->angle_increment;
    }
  }

  // Find the best point in the gap
  float best_idx = find_best_point(rdata, gap);

  // Publish Drive message
  float angle =
      scan_msg.get()->angle_min + best_idx * scan_msg.get()->angle_increment;

  // adjust speed based on angle
  float speed = 0.1;
  for (int idx = 0; idx < int(angle_ranges_.size()); ++idx) {
    if (abs(angle) < common::dtor(angle_ranges_.at(idx))) {
      speed = speed_ranges_.at(idx);
      break;
    }
  }

  // check if we are close to the wall
  // and update speed and angle to avoid collision
  auto wall_dist = check_wall_closeness(scan_msg);
  if (wall_dist.first < edge_distance_threshold_) {
    speed = 1.5;
    angle = 0;
    if (wall_dist.first < edge_distance_threshold_ * 0.7) {
      angle = common::dtor(15);
    }
  } else if (wall_dist.second < edge_distance_threshold_) {
    speed = 1.5;
    angle = 0;
    if (wall_dist.second < edge_distance_threshold_ * 0.7) {
      angle = common::dtor(-15);
    }
  }
  speed = std::min(max_speed_, speed);

  if (debug_ & 2) {
    RCLCPP_INFO(get_logger(),
                "Found Gap %d and best angle %f/%f min dist %f cnt %d "
                "zero_radius %d wd: [%f %f]",
                gap.second - gap.first, common::rtod(angle), speed, min_dist,
                int(min_dist_idx.size()), zero_radius, wall_dist.first,
                wall_dist.second);
  }

  // publish the drive message

  publish_drive_angle(angle, speed);

  if (debug_ & 1) {
    if (true) {  // draw the direction angle
      float cangle = scan_msg.get()->angle_min +
                     scan_msg.get()->angle_increment * best_idx;
      debug_lines_.at(DRIVE)->add_line(0, 0, rdata[best_idx] * cos(cangle),
                                       rdata[best_idx] * sin(cangle));
    }

    // update the debug lines on rviz
    for (auto dt : debug_lines_) {
      dt.second->send_msg(this->get_clock().get()->now());
    }
  }
}

/// \brief Publish a drive message to the /drive topic.
///
/// This function is called by the lidar callback to publish the desired
/// steering angle and speed to the /drive topic.
///
/// \param angle The desired steering angle in radians
/// \param speed The desired speed in m/s
void GapFollow::publish_drive_angle(float angle, float speed) {
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.drive.speed = (true) ? speed : 0.0;
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.steering_angle_velocity = 0.1;
  ack_pub_->publish(drive_msg);
}
}  // namespace gap_follow