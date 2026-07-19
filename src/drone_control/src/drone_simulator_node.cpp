#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <random>

class DroneSimulatorNode : public rclcpp::Node {
public:
    DroneSimulatorNode() : Node("drone_simulator_node"), altitude_(0.0), velocity_(0.0), thrust_(0.0) {
        raw_alt_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/altitude_raw", 10);
        
        thrust_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/thrust", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { thrust_ = msg->data; });
            
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DroneSimulatorNode::physics_loop, this));
            
        gen_.seed(rd_());
    }

private:
    void physics_loop() {
        double dt = 0.1;
        double gravity = 9.81;
        
        // Physics update
        altitude_ += velocity_ * dt;
        velocity_ += (thrust_ - gravity) * dt;

        // Ground constraint
        if (altitude_ < 0.0) { altitude_ = 0.0; velocity_ = 0.0; }

        // Add Gaussian noise to simulate a bad sensor
        std::normal_distribution<double> noise(0.0, 0.5);
        
        std_msgs::msg::Float64 alt_msg;
        alt_msg.data = altitude_ + noise(gen_);
        raw_alt_pub_->publish(alt_msg);
    }

    double altitude_, velocity_, thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_alt_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::random_device rd_;
    std::mt19937 gen_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}