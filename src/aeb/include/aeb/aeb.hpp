#ifndef AEB_HPP_
#define AEB_HPP_
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace aeb {

class AEB : public rclcpp::Node {
public:
    explicit AEB();
    ~AEB() = default;
    void init();
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

};

}
#endif