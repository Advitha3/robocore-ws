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
        // Declare the burst timing parameters
        this->declare_parameter("fault_burst_start", 10);
        this->declare_parameter("fault_burst_end", 15);

        publisher_ = this->create_publisher<sensor_interfaces::msg::SensorReading>("/sensor/imu", 10);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SensorPublisherNode::produce_and_publish, this));

        gen_.seed(rd_());
        dist_ = std::uniform_real_distribution<double>(-0.1, 0.1);
    }

private:
    void produce_and_publish() {
        cycle_count_++;

        InternalReading* reading = pool_.allocate();
        if (!reading) return;

        std::strncpy(reading->sensor_name, "IMU_Primary", sizeof(reading->sensor_name) - 1);
        reading->sensor_name[sizeof(reading->sensor_name) - 1] = '\0';
        
        reading->value = 9.81 + dist_(gen_);
        reading->timestamp = this->now().seconds();
        
        // Fetch current parameters on the fly
        int start = this->get_parameter("fault_burst_start").as_int();
        int end = this->get_parameter("fault_burst_end").as_int();

        // Use dynamic parameters for fault calculation
        reading->is_healthy = !(cycle_count_ % 50 >= start && cycle_count_ % 50 <= end);

        buffer_.push(reading);

        InternalReading* processed = nullptr;
        if (buffer_.pop(processed)) { 
            auto msg = sensor_interfaces::msg::SensorReading();
            msg.sensor_name = processed->sensor_name;
            msg.value = processed->value;
            msg.timestamp = processed->timestamp;
            msg.is_healthy = processed->is_healthy;

            publisher_->publish(msg);
            
            if (!msg.is_healthy) {
                 RCLCPP_WARN(this->get_logger(), "[POOL] Published FAULT: val=%.2f", msg.value);
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