#include "total_persuit/waypoint_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
namespace total_persuit {

WaypointPublisher::WaypointPublisher(rclcpp::Node::SharedPtr node)
    : node_(node) {
  publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "waypoints", 1000);
}

void WaypointPublisher::loadWaypoints(
    const std::vector<double>& waypoints_param) {
  waypoints_.resize(waypoints_param.size() / 2);
  // RCLCPP_INFO(node_->get_logger(), "Waypoint publisher got size %d.",
  //             waypoints_param.size());
  for (size_t i = 0; i < waypoints_param.size(); i += 2) {
    waypoints_[i / 2] = {waypoints_param[i], waypoints_param[i + 1]};
  }
}

void WaypointPublisher::publishWaypoint() {
  auto marker_array = visualization_msgs::msg::MarkerArray();
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.a = 1.0;
  marker.color.g = 1.0;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    marker.pose.position.x = waypoints_.at(i).at(0);
    marker.pose.position.y = waypoints_.at(i).at(1);
    marker.id = i;
    marker_array.markers.push_back(marker);
  }
  publisher_->publish(marker_array);
}

size_t WaypointPublisher::getWaypointCount() const { return waypoints_.size(); }

}  // namespace total_persuit
