#include <chrono>
#include <memory>
#include <cstring>
#include <random> // <-- Fix 2: Added the random header
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_reading.hpp"

#include "pool_allocator.hpp"
#include "circular_buffer.hpp"

using namespace std::chrono_literals;

struct InternalReading {
    char sensor_name[32]; // <-- Fix 1: Added the semicolon
    double value;
    double timestamp;
    bool is_healthy;
};

class SensorPublisherNode : public rclcpp::Node {
public:
    SensorPublisherNode() : Node("sensor_publisher_node"), cycle_count_(0) {
        publisher_ = this->create_publisher<sensor_interfaces::msg::SensorReading>("/sensor/imu", 10);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SensorPublisherNode::produce_and_publish, this));

        gen_.seed(rd_());
        dist_ = std::uniform_real_distribution<double>(-0.1, 0.1);

        RCLCPP_INFO(this->get_logger(), "Real-Time Sensor Publisher Node started.");
    }

private:
    void produce_and_publish() {
        cycle_count_++;

        // --- STEP A: Allocate ---
        InternalReading* reading = pool_.allocate();
        if (!reading) {
            RCLCPP_ERROR(this->get_logger(), "Pool allocator is full! Dropping data.");
            return;
        }

        // --- STEP B: Populate Simulated Data ---
        std::strncpy(reading->sensor_name, "IMU_Primary", sizeof(reading->sensor_name) - 1);
        reading->sensor_name[sizeof(reading->sensor_name) - 1] = '\0';
        
        reading->value = 9.81 + dist_(gen_);
        reading->timestamp = this->now().seconds();
        reading->is_healthy = (cycle_count_ % 5 != 0);

        // --- STEP C: Push to Circular Buffer ---
        buffer_.push(reading);

        // --- STEP D: Pop from Circular Buffer ---
        InternalReading* processed = nullptr; // Create an empty pointer
        
        // Fix 3: Pass the pointer by reference into your Day 1 pop() function
        if (buffer_.pop(processed)) { 
            
            // --- STEP E: Convert to ROS 2 Message ---
            auto msg = sensor_interfaces::msg::SensorReading();
            msg.sensor_name = processed->sensor_name;
            msg.value = processed->value;
            msg.timestamp = processed->timestamp;
            msg.is_healthy = processed->is_healthy;

            // --- STEP F: Publish ---
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "[POOL] Published: %s val=%.2f | Healthy: %s", 
                        msg.sensor_name.c_str(), msg.value, msg.is_healthy ? "TRUE" : "FALSE");

            // --- STEP H: Deallocate ---
            pool_.deallocate(processed);
        }
    }

    int cycle_count_;
    rclcpp::Publisher<sensor_interfaces::msg::SensorReading>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    PoolAllocator<InternalReading, 16> pool_;
    CircularBuffer<InternalReading*, 16> buffer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisherNode>());
    rclcpp::shutdown();
    return 0;
}