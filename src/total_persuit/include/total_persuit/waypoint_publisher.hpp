#ifndef WAYPOINT_PUBLISHER_HPP
#define WAYPOINT_PUBLISHER_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace total_persuit {

class WaypointPublisher {
 public:
  WaypointPublisher(rclcpp::Node::SharedPtr node);

  void loadWaypoints(const std::vector<double>& waypoints_param);
  void publishWaypoint();

  size_t getWaypointCount() const;
  const std::vector<std::array<double, 2>>& getWaypoints() {
    return waypoints_;
  }

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  std::vector<std::array<double, 2>> waypoints_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace total_persuit

#endif  // WAYPOINT_PUBLISHER_HPP
