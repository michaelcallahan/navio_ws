#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "navio/ublox.h"

class GpsNode : public rclcpp::Node
{
public:
    GpsNode() : Node("gps_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GpsNode::timer_callback, this));
        gps_.initialize();
    }

private:
    void timer_callback()
    {
        navio::UBlox::GPSData gps_data = gps_.read();
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = gps_data.latitude;
        msg.longitude = gps_data.longitude;
        msg.altitude = gps_data.altitude;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    navio::UBlox gps_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNode>());
    rclcpp::shutdown();
    return 0;
}