#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "navio/adafruit_adc.h"

class BatteryMonitorNode : public rclcpp::Node
{
public:
    BatteryMonitorNode() : Node("battery_monitor_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryMonitorNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        float battery_voltage = adc_.read_voltage();
        sensor_msgs::msg::BatteryState msg;
        msg.voltage = battery_voltage;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    navio::AdafruitADC adc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryMonitorNode>());
    rclcpp::shutdown();
    return 0;
}