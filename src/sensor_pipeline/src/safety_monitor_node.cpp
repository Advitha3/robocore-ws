#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_reading.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "safety_fsm.hpp"

class SafetyMonitorNode : public rclcpp::Node {
public:
    SafetyMonitorNode() : Node("safety_monitor_node") {
        // 1. Declare first to register with the parameter server
        this->declare_parameter("degraded_threshold", 3);
        this->declare_parameter("estop_threshold", 5);
        
        // 2. Get the actual values (incorporating launch overrides)
        int deg_thresh = this->get_parameter("degraded_threshold").as_int();
        int estop_thresh = this->get_parameter("estop_threshold").as_int();
        
        // 3. Initialize the FSM with the ROS parameters
        fsm_ = std::make_unique<drone_safety::SafetyFSM>(deg_thresh, estop_thresh);

        subscription_ = this->create_subscription<sensor_interfaces::msg::SensorReading>(
            "/sensor/imu", 10,
            std::bind(&SafetyMonitorNode::sensor_callback, this, std::placeholders::_1));
        
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/state", 10);

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/safety/reset",
            std::bind(&SafetyMonitorNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "FSM initialized: degrade=%d estop=%d", deg_thresh, estop_thresh);
    }

private:
    void sensor_callback(const sensor_interfaces::msg::SensorReading::SharedPtr msg) {
        drone_safety::SafetyState current_state = fsm_->update(msg->is_healthy);

        auto state_msg = std_msgs::msg::String();
        state_msg.data = fsm_->get_state_string();
        state_pub_->publish(state_msg);

        switch (current_state) {
            case drone_safety::SafetyState::NORMAL_OP:
                RCLCPP_INFO(this->get_logger(), "[OK] state=NORMAL | faults=%d/%d", 
                            fsm_->get_consecutive_faults(), fsm_->get_total_faults());
                break;
            case drone_safety::SafetyState::DEGRADED:
                RCLCPP_WARN(this->get_logger(), "[DEGRADED] consecutive=%d — reducing thrust", 
                            fsm_->get_consecutive_faults());
                break;
            case drone_safety::SafetyState::EMERGENCY_STOP:
                RCLCPP_ERROR(this->get_logger(), "[E-STOP] HARD LATCH — manual reset required");
                break;
        }
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        RCLCPP_INFO(this->get_logger(), "Received Supervisory Reset command!");
        fsm_->supervisory_reset();
        response->success = true;
        response->message = "FSM successfully reset to NORMAL_OP.";
    }
    
    rclcpp::Subscription<sensor_interfaces::msg::SensorReading>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    
    std::unique_ptr<drone_safety::SafetyFSM> fsm_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitorNode>());
    rclcpp::shutdown();
    return 0;
}