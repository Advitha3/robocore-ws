#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "safety_fsm.hpp"

class SafetyMonitorNode : public rclcpp::Node {
public:
    SafetyMonitorNode() : Node("safety_monitor_node") {
        this->declare_parameter("degraded_threshold", 3);
        this->declare_parameter("estop_threshold", 5);
        
        int deg_thresh = this->get_parameter("degraded_threshold").as_int();
        int estop_thresh = this->get_parameter("estop_threshold").as_int();
        
        fsm_ = std::make_unique<drone_safety::SafetyFSM>(deg_thresh, estop_thresh);

        // Subscribe to the filtered altitude now
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude_filtered", 10,
            std::bind(&SafetyMonitorNode::altitude_callback, this, std::placeholders::_1));
        
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/state", 10);
        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/safety/reset",
            std::bind(&SafetyMonitorNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "FSM initialized: degrade=%d estop=%d", deg_thresh, estop_thresh);
    }

private:
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Bounds checking: Altitude shouldn't cross 15m or drop below 0m
        bool is_healthy = !(msg->data > 15.0 || msg->data < 0.0);
        
        drone_safety::SafetyState current_state = fsm_->update(is_healthy);

        auto state_msg = std_msgs::msg::String();
        state_msg.data = fsm_->get_state_string();
        state_pub_->publish(state_msg);

        switch (current_state) {
            case drone_safety::SafetyState::NORMAL_OP:
                // Silent on normal to reduce log spam
                break;
            case drone_safety::SafetyState::DEGRADED:
                RCLCPP_WARN(this->get_logger(), "[DEGRADED] Altitude out of bounds! consecutive=%d", 
                            fsm_->get_consecutive_faults());
                break;
            case drone_safety::SafetyState::EMERGENCY_STOP:
                RCLCPP_ERROR(this->get_logger(), "[E-STOP] HARD LATCH — Altitude severely out of bounds!");
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
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
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