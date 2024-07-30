
#include <aeb/aeb.hpp>


namespace aeb {
AEB::AEB()
: Node("Automatic_Emergency_Breaking", "")
{}

void AEB::init() {
    RCLCPP_INFO(this->get_logger(), "Init!");
    // subascribe to topic
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,// rclcpp::QoS(10),
        std::bind(&AEB::scan_callback, this, std::placeholders::_1));
//    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("racecar/odom", 10, std::bind(&AEB::odom_callback, this, _1));
  // subscribe to ackermann topic
//  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
  //  "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}
void AEB::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    
    RCLCPP_INFO(get_logger(), "got message: angle:[%f %f %f] range:%d", 
        msg.get()->angle_min, 
        msg.get()->angle_max, 
        msg.get()->angle_increment, 
    (int)msg.get()->ranges.size());

}

}