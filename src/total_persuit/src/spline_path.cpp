#include "total_persuit/spline_path.hpp"

#include "lib/common.hpp"
namespace total_persuit {
using Point = std::pair<double, double>;

// Helper function to calculate Euclidean distance
static double euclideanDistance(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p2.first - p1.first, 2) +
                   std::pow(p2.second - p1.second, 2));
}
// Method to find the closest point at a given distance from the input point
Point findClosestPointAtDistance(
    rclcpp::Node::SharedPtr node_,
    const std::vector<std::array<double, 2>>& spline, double x, double y,
    double yaw, double approx_dist, double& out_angle) {
  // Find the closest point on the spline
  double minDistance = std::numeric_limits<double>::max();
  size_t closestIndex = 0;
  size_t closest_forward = 0;
  // RCLCPP_INFO(node_->get_logger(),
  //             "Starting Search car at x,y => %f %f yaw = %f", x, y,
  //             common::rtod(yaw));
  for (size_t i = 0; i < spline.size(); ++i) {
    double dist =
        euclideanDistance(Point(x, y), Point(spline[i][0], spline[i][1]));
    if (dist < minDistance && dist >= approx_dist) {
      double angle2 = std::atan((y - spline[i][1]) / (x - spline[i][0]));
      double angle = angle2;
      if (spline[i][0] < x) {
        angle = common::dtor(180) + angle;
      }
      // RCLCPP_INFO(node_->get_logger(),
      //             " point=> %f,%f angel = %f/%f idx = %d dist = %f",
      //             spline[i][0], spline[i][1], common::rtod(angle),
      //             common::rtod(angle2), i, dist);

      double d = abs(angle - yaw);
      if (d > common::dtor(360)) d -= common::dtor(360);
      d = std::min(d, common::dtor(360) - d);

      if (d < common::dtor(90.0)) {
        minDistance = dist;
        closestIndex = i;
        out_angle = angle;
      }
    }
  }
  return Point(spline[closestIndex][0], spline[closestIndex][1]);
}

SplinePath::SplinePath(rclcpp::Node::SharedPtr node) : node_(node) {
  publisher_ = node_->create_publisher<nav_msgs::msg::Path>("spline_path", 10);
}
void SplinePath::generate(const std::vector<std::array<double, 2>>& waypoints) {
  if (waypoints.size() < 2) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Not enough waypoints to generate a spline.");
    return;
  }
  interpolated_points_ = generateSpline(waypoints);
}

void SplinePath::publish() {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = node_->get_clock()->now();

  for (const auto& point : interpolated_points_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->get_clock()->now();
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  publisher_->publish(path_msg);
  // RCLCPP_INFO(node_->get_logger(), "Published spline path with %zu points.",
  //             interpolated_points_.size());
}

std::vector<std::array<double, 2>> SplinePath::generateSpline(
    const std::vector<std::array<double, 2>>& waypoints) {
  std::vector<std::array<double, 2>> interpolated_points;
  waypoints_ = waypoints;
  const size_t n = waypoints.size();
  const int num_intermediate_points = 20;

  for (size_t i = 0; i < n - 1; ++i) {
    auto p0 = waypoints[i == 0 ? i : i - 1];
    auto p1 = waypoints[i];
    auto p2 = waypoints[i + 1];
    auto p3 = waypoints[i + 2 < n ? i + 2 : i + 1];

    for (int j = 0; j < num_intermediate_points; ++j) {
      double t = static_cast<double>(j) / num_intermediate_points;
      double t2 = t * t;
      double t3 = t2 * t;

      double x =
          0.5 * ((-t3 + 2 * t2 - t) * p0[0] + (3 * t3 - 5 * t2 + 2) * p1[0] +
                 (-3 * t3 + 4 * t2 + t) * p2[0] + (t3 - t2) * p3[0]);
      double y =
          0.5 * ((-t3 + 2 * t2 - t) * p0[1] + (3 * t3 - 5 * t2 + 2) * p1[1] +
                 (-3 * t3 + 4 * t2 + t) * p2[1] + (t3 - t2) * p3[1]);

      interpolated_points.push_back({x, y});
    }
  }

  interpolated_points.push_back({waypoints.back()[0], waypoints.back()[1]});

  return interpolated_points;
}
void SplinePath::get_point_on_path(double in_x, double in_y, double yaw,
                                   double dist, double& out_x, double& out_y,
                                   double& out_angle) {
  //

  // Point findClosestPointAtDistance(
  //   const std::vector<std::array<double, 2>>& spline, double x, double y,
  //   double approx_dist, bool direction)
  auto res = findClosestPointAtDistance(node_,
                                        // waypoints_,
                                        interpolated_points_, in_x, in_y, yaw,
                                        dist, out_angle);
  out_x = res.first;
  out_y = res.second;
}

}  // namespace total_persuit
