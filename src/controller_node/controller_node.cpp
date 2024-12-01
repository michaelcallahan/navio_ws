#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_msgs/msg/state_estimate.hpp"
#include "custom_msgs/msg/motor_control.hpp"
#include <Eigen/Dense>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node"), K_()
    {
        state_subscription_ = this->create_subscription<custom_msgs::msg::StateEstimate>("state/estimate", 10, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));
        command_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("command", 10, std::bind(&ControllerNode::command_callback, this, std::placeholders::_1));
        motor_publisher_ = this->create_publisher<custom_msgs::msg::MotorControl>("motor/control", 10);

        // Initialize LQR gain matrix K
        K_ << 1.5, 0.0, 0.0,
              0.0, 1.5, 0.0,
              0.0, 0.0, 1.2;
    }

private:
    void state_callback(const custom_msgs::msg::StateEstimate::SharedPtr msg)
    {
        // Store the current state for use in control calculations
        current_state_ = *msg;
        compute_control();
    }

    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Store the command for use in control calculations
        current_command_ = *msg;
    }

    void compute_control()
    {
        if (current_command_) {
            // Calculate the error between the desired state and current state
            Eigen::Vector3d desired_velocity(current_command_->linear.x, current_command_->linear.y, current_command_->linear.z);
            Eigen::Vector3d current_velocity(current_state_.vx, current_state_.vy, current_state_.vz);
            Eigen::Vector3d error = current_velocity - desired_velocity;

            // Compute control input using LQR: u = -K * error
            Eigen::Vector3d control_output = -K_ * error;

            // Create and publish MotorControl message
            custom_msgs::msg::MotorControl motor_msg;
            motor_msg.thrust = control_output(2);  // Throttle (up/down)
            motor_msg.roll = control_output(0);    // Roll (lateral/side-to-side)
            motor_msg.pitch = control_output(1);   // Pitch (forward/backward)
            motor_msg.yaw = current_command_->angular.z;

            motor_publisher_->publish(motor_msg);
        }
    }

    rclcpp::Subscription<custom_msgs::msg::StateEstimate>::SharedPtr state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
    rclcpp::Publisher<custom_msgs::msg::MotorControl>::SharedPtr motor_publisher_;
    custom_msgs::msg::StateEstimate current_state_;
    geometry_msgs::msg::Twist::SharedPtr current_command_;
    Eigen::Matrix3d K_;  // LQR gain matrix
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}