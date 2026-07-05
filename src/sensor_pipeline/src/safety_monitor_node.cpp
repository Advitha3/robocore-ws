#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_reading.hpp"

class SafetyMonitorNode : public rclcpp::Node {
public:
    SafetyMonitorNode() : Node("safety_monitor_node") {
        // Subscribe to the custom IMU topic
        subscription_ = this->create_subscription<sensor_interfaces::msg::SensorReading>(
            "/sensor/imu", 10,
            std::bind(&SafetyMonitorNode::sensor_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Safety Monitor active. Watching for sensor faults...");
    }

private:
    void sensor_callback(const sensor_interfaces::msg::SensorReading::SharedPtr msg) {
        // Read the custom message data
        if (msg->is_healthy) {
            // Standard logging for healthy data (using debug or info)
            RCLCPP_INFO(this->get_logger(), "[OK] %s reading normal: %.2f", 
                        msg->sensor_name.c_str(), msg->value);
        } else {
            // Trigger a high-priority warning if the sensor fails
            RCLCPP_WARN(this->get_logger(), "[CRITICAL FAULT] %s reported failure at %.2f! Initiating emergency protocol.", 
                        msg->sensor_name.c_str(), msg->value);
        }
    }
    
    rclcpp::Subscription<sensor_interfaces::msg::SensorReading>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitorNode>());
    rclcpp::shutdown();
    return 0;
}