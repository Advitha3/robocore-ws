#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// 1. Your Week 2 Kalman Filter Class
class KalmanFilter1D {
public:
    double x = 0.0; // State estimate (Altitude)
    double P = 1.0; // Estimate uncertainty
    double Q = 0.1; // Process noise (Physics unpredictability)
    double R = 0.5; // Measurement noise (Sensor inaccuracy)

    void predict(double u, double dt) {
        x = x + u * dt;
        P = P + Q;
    }

    void update(double z) {
        double K = P / (P + R);
        x = x + K * (z - x);
        P = (1 - K) * P;
    }
};

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node") {
        // Subscribe to noisy raw altitude
        sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude_raw", 10,
            std::bind(&EKFNode::altitude_callback, this, std::placeholders::_1));

        // Publish clean filtered altitude
        pub_ = this->create_publisher<std_msgs::msg::Float64>("/drone/altitude_filtered", 10);
    }

private:
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // 1. Predict (assuming slight upward velocity for the model)
        ekf_.predict(0.5, 0.1); 

        // 2. Update with the raw, noisy measurement
        ekf_.update(msg->data);

        // 3. Publish the clean estimate
        std_msgs::msg::Float64 filtered_msg;
        filtered_msg.data = ekf_.x;
        pub_->publish(filtered_msg);

        // 4. Log the comparison
        RCLCPP_INFO(this->get_logger(), "Raw: %.2f | Filtered: %.2f | P: %.3f", 
                    msg->data, ekf_.x, ekf_.P);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    KalmanFilter1D ekf_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}