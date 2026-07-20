#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <numeric>
#include <cmath>

class BagAnalyzerNode : public rclcpp::Node {
public:
    BagAnalyzerNode() : Node("bag_analyzer_node") {
        raw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude_raw", 10,
            std::bind(&BagAnalyzerNode::raw_callback, this, std::placeholders::_1));

        filtered_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/altitude_filtered", 10,
            std::bind(&BagAnalyzerNode::filtered_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Bag Analyzer started. Waiting for data...");
    }

private:
    void raw_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        raw_buffer_.push_back(msg->data);
    }

    void filtered_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        filtered_buffer_.push_back(msg->data);

        // Every 10 messages, compute the statistics
        if (filtered_buffer_.size() >= 10 && raw_buffer_.size() >= 10) {
            compute_and_print();
            
            // Clear the buffers for the next batch
            raw_buffer_.clear();
            filtered_buffer_.clear();
        }
    }

    double mean(const std::vector<double>& v) {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        return sum / v.size();
    }

    double stddev(const std::vector<double>& v, double mean_val) {
        double variance = 0.0;
        for (double val : v) {
            variance += (val - mean_val) * (val - mean_val);
        }
        return std::sqrt(variance / v.size());
    }

    void compute_and_print() {
        double r_mean = mean(raw_buffer_);
        double r_std = stddev(raw_buffer_, r_mean);

        double f_mean = mean(filtered_buffer_);
        double f_std = stddev(filtered_buffer_, f_mean);

        double reduction = 0.0;
        if (r_std > 0.0) {
            reduction = ((r_std - f_std) / r_std) * 100.0;
        }

        // Print the statistical proof!
        RCLCPP_INFO(this->get_logger(),
                    "[STATS] Raw \u03c3=%.3f | Filtered \u03c3=%.3f | Noise reduction: %.1f%%",
                    r_std, f_std, reduction);
    }

    std::vector<double> raw_buffer_;
    std::vector<double> filtered_buffer_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr raw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr filtered_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagAnalyzerNode>());
    rclcpp::shutdown();
    return 0;
}