#ifndef SAFETY_FSM_HPP
#define SAFETY_FSM_HPP

#include <string>
#include <cstdint>

namespace drone_safety {

enum class SafetyState {
    NORMAL_OP,
    DEGRADED,
    EMERGENCY_STOP
};

class SafetyFSM {
public:
    /**
     * @param degraded_threshold Number of consecutive faults before degrading performance.
     * @param estop_threshold Number of consecutive faults before initiating Emergency Stop.
     */
    explicit SafetyFSM(uint32_t degraded_threshold = 3, uint32_t estop_threshold = 5)
        : current_state_(SafetyState::NORMAL_OP),
          consecutive_faults_(0),
          total_faults_(0),
          total_readings_(0),
          degraded_threshold_(degraded_threshold),
          estop_threshold_(estop_threshold) {}

    /**
     * @brief Processes a heartbeat or health reading from the sensor pipeline.
     * @param is_healthy True if the sensor reading passed validation.
     * @return The updated safety state.
     */
    SafetyState update(bool is_healthy) {
        total_readings_++;

        if (!is_healthy) {
            consecutive_faults_++;
            total_faults_++;
        } else {
            // Healthy reading resets consecutive fault counter
            consecutive_faults_ = 0;
        }

        evaluate_state_transitions();
        return current_state_;
    }

    /**
     * @brief Manually resets the state machine back to NORMAL_OP (e.g., after ground inspection).
     */
    void supervisory_reset() {
        current_state_ = SafetyState::NORMAL_OP;
        consecutive_faults_ = 0;
    }

    // Getters
    SafetyState get_state() const { return current_state_; }
    uint32_t get_consecutive_faults() const { return consecutive_faults_; }
    uint32_t get_total_faults() const { return total_faults_; }

    std::string get_state_string() const {
        switch (current_state_) {
            case SafetyState::NORMAL_OP:      return "NORMAL_OP";
            case SafetyState::DEGRADED:       return "DEGRADED";
            case SafetyState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default:                          return "UNKNOWN";
        }
    }

private:
    void evaluate_state_transitions() {
        // Latch behavior: Emergency Stop cannot be cleared by transient healthy readings alone
        if (current_state_ == SafetyState::EMERGENCY_STOP) {
            return;
        }

        if (consecutive_faults_ >= estop_threshold_) {
            current_state_ = SafetyState::EMERGENCY_STOP;
        } else if (consecutive_faults_ >= degraded_threshold_) {
            current_state_ = SafetyState::DEGRADED;
        } else if (consecutive_faults_ == 0 && current_state_ == SafetyState::DEGRADED) {
            // Auto-recovery to normal if healthy readings return while in degraded state
            current_state_ = SafetyState::NORMAL_OP;
        }
    }

    SafetyState current_state_;
    uint32_t consecutive_faults_;
    uint32_t total_faults_;
    uint32_t total_readings_;
    const uint32_t degraded_threshold_;
    const uint32_t estop_threshold_;
};

} // namespace drone_safety

#endif // SAFETY_FSM_HPP