#include <chrono>
#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class DroneSimulatorNode : public rclcpp::Node {
public:
    DroneSimulatorNode() : Node("drone_simulator_node"), 
                           altitude_(0.0), 
                           velocity_(0.0), 
                           current_thrust_(0.0),
                           gravity_(9.8) 
    {
        // 1. Subscribe to Thrust (Listen to the Brain)
        thrust_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/thrust", 10,
            std::bind(&DroneSimulatorNode::thrust_callback, this, std::placeholders::_1)
        );

        // 2. Publish Altitude (Send sensor data to the Brain)
        altitude_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/altitude", 10);

        // 3. Timer for the physics loop (10Hz -> dt = 0.1)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&DroneSimulatorNode::physics_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Drone Physics Simulator has been started.");
    }

private:
    double altitude_;
    double velocity_;
    double current_thrust_;
    double gravity_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Update internal thrust variable when the PID node sends a command
    void thrust_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_thrust_ = msg->data;
    }

    // Run the physics engine
    void physics_loop() {
        double dt = 0.1;

        // 1. Calculate physics
        velocity_ += (current_thrust_ - gravity_) * dt;
        altitude_ += velocity_ * dt;

        // 2. The Ground Check (Can't fall through the floor)
        if (altitude_ <= 0.0) {
            altitude_ = 0.0;
            
            // If thrust isn't strong enough to lift off, zero out velocity
            if (current_thrust_ <= gravity_) {
                velocity_ = 0.0; 
            }
        }

        // 3. Publish the new altitude
        auto alt_msg = std_msgs::msg::Float64();
        alt_msg.data = altitude_;
        altitude_pub_->publish(alt_msg);

        // Log it so we can watch it fly
        RCLCPP_INFO(this->get_logger(), "Sim Altitude: %.2fm | Current Thrust: %.2f", altitude_, current_thrust_);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}