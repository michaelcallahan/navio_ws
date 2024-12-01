#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "custom_msgs/msg/state_estimate.hpp"
#include <Eigen/Dense>

class StateEstimatorNode : public rclcpp::Node
{
public:
    StateEstimatorNode() : Node("state_estimator_node")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&StateEstimatorNode::imu_callback, this, std::placeholders::_1));
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, std::bind(&StateEstimatorNode::gps_callback, this, std::placeholders::_1));
        state_publisher_ = this->create_publisher<custom_msgs::msg::StateEstimate>("state/estimate", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Process IMU data
        imu_acc_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        imu_gyro_ = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        estimate_state();
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Process GPS data
        gps_position_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
        estimate_state();
    }

    void estimate_state()
    {
        // Simple state estimation logic (can be replaced by a more sophisticated filter)
        custom_msgs::msg::StateEstimate state_msg;
        state_msg.x = gps_position_(0);  // Placeholder for actual state estimation logic
        state_msg.y = gps_position_(1);
        state_msg.z = gps_position_(2);
        state_msg.vx = imu_acc_(0);  // Placeholder for velocity estimation
        state_msg.vy = imu_acc_(1);
        state_msg.vz = imu_acc_(2);
        state_publisher_->publish(state_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Publisher<custom_msgs::msg::StateEstimate>::SharedPtr state_publisher_;
    Eigen::Vector3d imu_acc_;
    Eigen::Vector3d imu_gyro_;
    Eigen::Vector3d gps_position_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}