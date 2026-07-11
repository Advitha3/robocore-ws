#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_reading.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

// Include your custom FSM Header
#include "safety_fsm.hpp"

class SafetyMonitorNode : public rclcpp::Node {
public:
    SafetyMonitorNode() : Node("safety_monitor_node"), fsm_(3, 5) {
        
        // 1. Subscribe to the raw IMU data
        subscription_ = this->create_subscription<sensor_interfaces::msg::SensorReading>(
            "/sensor/imu", 10,
            std::bind(&SafetyMonitorNode::sensor_callback, this, std::placeholders::_1));
        
        // 2. Publish the calculated State Machine string
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/state", 10);

        // 3. Expose the Supervisory Reset Service
        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/safety/reset",
            std::bind(&SafetyMonitorNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized. FSM [Degrade: 3, E-STOP: 5].");
    }

private:
    void sensor_callback(const sensor_interfaces::msg::SensorReading::SharedPtr msg) {
        
        // 1. Advance the State Machine
        drone_safety::SafetyState current_state = fsm_.update(msg->is_healthy);

        // 2. Broadcast the state to the rest of the ROS 2 network
        auto state_msg = std_msgs::msg::String();
        state_msg.data = fsm_.get_state_string();
        state_pub_->publish(state_msg);

        // 3. React locally
        switch (current_state) {
            case drone_safety::SafetyState::NORMAL_OP:
                RCLCPP_INFO(this->get_logger(), "[OK] state=NORMAL | faults=%d/%d", 
                            fsm_.get_consecutive_faults(), fsm_.get_total_faults());
                break;
            
            case drone_safety::SafetyState::DEGRADED:
                RCLCPP_WARN(this->get_logger(), "[DEGRADED] consecutive=%d — reducing thrust", 
                            fsm_.get_consecutive_faults());
                break;
            
            case drone_safety::SafetyState::EMERGENCY_STOP:
                RCLCPP_ERROR(this->get_logger(), "[E-STOP] HARD LATCH — manual reset required");
                break;
        }
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Suppress unused parameter warning
        (void)request;

        RCLCPP_INFO(this->get_logger(), "Received Supervisory Reset command from network!");
        
        // Reset the FSM
        fsm_.supervisory_reset();
        
        // Reply to whoever called the service
        response->success = true;
        response->message = "FSM successfully reset to NORMAL_OP.";
    }
    
    rclcpp::Subscription<sensor_interfaces::msg::SensorReading>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    
    drone_safety::SafetyFSM fsm_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitorNode>());
    rclcpp::shutdown();
    return 0;
}