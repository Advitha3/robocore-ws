#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PIDNode : public rclcpp::Node {
public:
    PIDNode() : Node("pid_node"), integral_(0.0), prev_error_(0.0) {
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/thrust", 10);
        
        // Subscribe to FILTERED altitude (clean data)
        alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude_filtered", 10,
            std::bind(&PIDNode::altitude_callback, this, std::placeholders::_1));
    }

private:
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double target = 10.0;
        double dt = 0.1;
        double kp = 2.0, ki = 0.2, kd = 1.5;

        // PID Math
        double error = target - msg->data;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        // Thrust computation with gravity feed-forward
        double thrust = (kp * error) + (ki * integral_) + (kd * derivative) + 9.81;
        
        // Clamp thrust
        if (thrust < 0.0) thrust = 0.0;
        if (thrust > 20.0) thrust = 20.0;

        std_msgs::msg::Float64 t_msg;
        t_msg.data = thrust;
        thrust_pub_->publish(t_msg);
    }

    double integral_, prev_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNode>());
    rclcpp::shutdown();
    return 0;
}