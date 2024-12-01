#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "navio/mpu9250.h"

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ImuNode::timer_callback, this));
        imu_.initialize();
    }

private:
    void timer_callback()
    {
        float imu_data[9];
        imu_.getMotion9(imu_data);
        sensor_msgs::msg::Imu msg;
        msg.linear_acceleration.x = imu_data[0];
        msg.linear_acceleration.y = imu_data[1];
        msg.linear_acceleration.z = imu_data[2];
        msg.angular_velocity.x = imu_data[3];
        msg.angular_velocity.y = imu_data[4];
        msg.angular_velocity.z = imu_data[5];
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    navio::MPU9250 imu_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}