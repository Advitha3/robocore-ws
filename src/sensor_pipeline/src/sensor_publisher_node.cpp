#include <chrono>
#include <memory>
#include <cstring>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_reading.hpp"

#include "pool_allocator.hpp"
#include "circular_buffer.hpp"

using namespace std::chrono_literals;

struct InternalReading {
    char sensor_name[32];
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

        InternalReading* reading = pool_.allocate();
        if (!reading) {
            RCLCPP_ERROR(this->get_logger(), "Pool allocator is full! Dropping data.");
            return;
        }

        std::strncpy(reading->sensor_name, "IMU_Primary", sizeof(reading->sensor_name) - 1);
        reading->sensor_name[sizeof(reading->sensor_name) - 1] = '\0';
        
        reading->value = 9.81 + dist_(gen_);
        reading->timestamp = this->now().seconds();
        
        // Use a repeating burst pattern: Cycles 10 to 15 out of every 50 will be faults.
        // This allows you to test the E-STOP and Reset service multiple times!
        reading->is_healthy = !(cycle_count_ % 50 >= 10 && cycle_count_ % 50 <= 15);

        buffer_.push(reading);

        InternalReading* processed = nullptr;
        
        if (buffer_.pop(processed)) { 
            auto msg = sensor_interfaces::msg::SensorReading();
            msg.sensor_name = processed->sensor_name;
            msg.value = processed->value;
            msg.timestamp = processed->timestamp;
            msg.is_healthy = processed->is_healthy;

            publisher_->publish(msg);

            // Log so we can see when the publisher is sending faults
            if (msg.is_healthy) {
                 RCLCPP_INFO(this->get_logger(), "[POOL] Published: %s val=%.2f | Healthy: TRUE", msg.sensor_name.c_str(), msg.value);
            } else {
                 RCLCPP_WARN(this->get_logger(), "[POOL] Published: %s val=%.2f | Healthy: FALSE (FAULT INJECTED)", msg.sensor_name.c_str(), msg.value);
            }

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