#include <chrono>
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// 1. Reuse the PID Controller from Day 4
class PIDController {
private:
    double Kp, Ki, Kd;
    double integral_sum = 0.0;
    double previous_error = 0.0;

public:
    PIDController(double kp, double ki, double kd) 
        : Kp(kp), Ki(ki), Kd(kd) {}

    double compute(double target, double current, double dt) {
        double error = target - current;
        integral_sum += error * dt;
        double derivative = (error - previous_error) / dt;
        double output = Kp * error + Ki * integral_sum + Kd * derivative;
        previous_error = error;
        return output;
    }
};

// 2. The ROS 2 Node
class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode() : Node("pid_controller_node"), 
                          pid(3.0, 0.1, 2.0), // Using Tuning 5 (The Sweet Spot)
                          current_altitude(0.0), 
                          target_altitude(10.0) 
    {
        // Create Publisher for thrust
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/thrust", 10);

        // Create Subscriber for altitude
        altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude", 10,
            std::bind(&PIDControllerNode::altitude_callback, this, std::placeholders::_1)
        );

        // Create Timer at 10Hz (100ms) to run the control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PIDControllerNode::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "PID Controller Node has been started.");
    }

private:
    // Member variables
    PIDController pid;
    double current_altitude;
    double target_altitude;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback to update current altitude when a new message arrives
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_altitude = msg->data;
    }

    // 10Hz Control Loop
    void control_loop() {
        double dt = 0.1; // 10Hz = 0.1 seconds per cycle
        
        // Compute thrust
        double thrust = pid.compute(target_altitude, current_altitude, dt);

        // Clamp thrust between 0 and 20
        thrust = std::clamp(thrust, 0.0, 20.0);

        // Publish thrust
        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = thrust;
        thrust_pub_->publish(thrust_msg);

        // Log the output
        RCLCPP_INFO(this->get_logger(), "Altitude: %.2f | Thrust: %.2f", current_altitude, thrust);
    }
};

// 3. Main Execution
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    // Create the node and spin it so it stays alive
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}