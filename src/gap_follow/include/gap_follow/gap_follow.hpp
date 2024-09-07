#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "debug/debug_line.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace gap_follow {

/**
 * @brief Enum for the debug lines
 *
 * This enum is used to enable or disable the drawing of the different debug
 * lines.
 */
enum DEBUGLINES {
  /**
   * @brief Draw the longest sequence of free space points
   */
  GAPS = 1,
  /**
   * @brief Draw the angle of the car
   */
  SHORTEST = 2,
  /**
   * @brief Draw the direction of the car
   */
  DRIVE = 3,
  /**
   * @brief Draw the original scan data
   */
  SCAN = 4
};

class GapFollow : public rclcpp::Node {
 public:
  /**
   * @brief Construct a GapFollow node
   */
  explicit GapFollow();

  /**
   * @brief Initialize the node
   *
   * This function initializes the node by subscribing to the lidar scan topic
   * and advertising on the drive topic.
   */
  void init();

 private:
  /// Topic name for the lidar scan message.
  ///
  /// This message is a sensor_msgs::msg::LaserScan message.
  std::string lidarscan_topic = "/scan";

  /// Topic name for the drive message.
  ///
  /// This message is an ackermann_msgs::msg::AckermannDriveStamped message.
  std::string drive_topic = "/drive";

  /// Subscription to the lidar scan message.
  ///
  /// This subscription is used to receive the lidar scan message and call the
  /// lidar_callback() method.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  /// Subscription to the odometry message.
  ///
  /// This subscription is used to receive the odometry message and call the
  /// odom_callback() method.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  /// Publisher for the drive message.
  ///
  /// This publisher is used to send the drive message to the vehicle.
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ack_pub_;
  /// Current velocity of the vehicle.
  float current_velocity_;

  /// Previous error value for derivative control
  float prev_error_;

  /// Current error value for proportional control
  float error_;

  /// Integral error value for integral control
  float integral_;

  /// Maximum distance threshold for LiDAR scan data
  float max_dist_threshold_;

  /// Minimum distance threshold for LiDAR scan data
  float min_dist_threshold_;

  /// Drivable angle range for the vehicle in degrees
  float drivable_angle_;

  /// Size of the window for averaging the LiDAR scan ranges
  int range_average_window_size_;

  /// Car radius in meters
  float car_radius_;

  /// Disparity threshold for detecting jumps in the LiDAR scan
  float disparity_threshold_;

  /// Edge distance threshold for rejecting close points
  float edge_distance_threshold_;

  /// Maximum speed of the vehicle in meters per second
  float max_speed_;

  /// Angle ranges for the vehicle
  std::vector<double> angle_ranges_;

  /// Speed ranges for the vehicle
  std::vector<double> speed_ranges_;

  /// Debug level for the node
  int debug_;

  /// A map of debug lines that can be used for visualizing the LiDAR scan
  /**
   * A map of debug lines that can be used for visualizing the LiDAR scan.
   * The keys are the names of the debug lines.
   * The values are the debug lines themselves.
   */
  std::map<DEBUGLINES, std::shared_ptr<debug::DebugLine>> debug_lines_;

  /**
   * Callback for the LiDAR sensor.
   *
   * This function will be called whenever the LiDAR sensor sends a new scan.
   * The function will preprocess the LiDAR data, find the max gap, find the
   * best point in the gap and publish a drive message with the best angle and
   * speed.
   *
   * @param msg a shared pointer to a sensor_msgs::msg::LaserScan message
   */
  void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /**
   * Callback for the odometry message.
   *
   * This function will be called whenever the odometry message is received.
   * The function will update the current velocity of the vehicle.
   *
   * @param msg a shared pointer to a nav_msgs::msg::Odometry message
   */
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * Preprocess the LiDAR data.
   *
   * This function will preprocess the LiDAR data by rejecting high values and
   * calculating the mean over some window.
   *
   * @param ranges the LiDAR scan ranges
   * @param rdata the preprocessed LiDAR scan ranges
   * @param closest_points the indices of the closest points to the vehicle
   * @param drivable_angle_idx the starting index of the drivable angle range
   * @param angle_increment the angle increment of the LiDAR scan
   * @return the min distance point
   */
  float preprocess_lidar(const std::vector<float>& ranges,
                         std::vector<float>& rdata,
                         std::vector<int>& closest_points,
                         int drivable_angle_idx, float angle_increment);

  /**
   * Find the max gap in the LiDAR data.
   *
   * This function will find the max gap in the LiDAR data by iterating over the
   * ranges and find the longest sequence of points that are drivable.
   *
   * @param ranges the preprocessed LiDAR scan ranges
   * @param gap the start and end index of the max gap
   * @param drivable_angle_idx the starting index of the drivable angle range
   */
  void find_max_gap(const std::vector<float>& ranges, std::pair<int, int>& gap,
                    int drivable_angle_idx);

  /**
   * Find the best point in the gap.
   *
   * This function will find the best point in the gap by iterating over the
   * gap and find the longest sequence of points that are drivable.
   *
   * @param rdata the preprocessed LiDAR scan ranges
   * @param gap the start and end index of the max gap
   * @return the index of the best point in the gap
   */
  int find_best_point(std::vector<float>& rdata, std::pair<int, int>& gap);

  /**
   * Publish a drive message with the best angle and speed.
   *
   * This function will publish a drive message with the best angle and speed.
   *
   * @param angle the angle to publish
   * @param speed the speed to publish
   */
  void publish_drive_angle(float angle, float speed);

  /**
   * Check how close we are to the wall.
   *
   * This function will check how close we are to the wall by iterating over the
   * LiDAR scan and find the closest distance to the wall on the left and right
   * sides of the car.
   *
   * @param dt a shared pointer to a sensor_msgs::msg::LaserScan message
   * @return a pair of floats (left, right) representing the closest distance to
   * the wall on the left and right sides of the car
   */
  std::pair<float, float> check_wall_closeness(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr dt);
};

}  // namespace gap_follow