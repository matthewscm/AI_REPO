#include <memory>
#include <vector>
#include <string>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class BottleClassifierNode : public rclcpp::Node {
public:
    BottleClassifierNode() : Node("bottle_classifier_node") {
        // 1. Setup paths and Model Loading
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        load_model(pkg_share + "/models/bottle_svm_model.xml", pkg_share + "/models/scaling_params.xml");

        // 2. Pub/Sub
        feature_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bottle_features", 10, std::bind(&BottleClassifierNode::feature_callback, this, std::placeholders::_1));
        
        recyclable_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_recyclable", 10);

        // 3. Timers
        // Update the 'last_decision_' every 5 seconds based on accumulated features
        update_timer_ = this->create_wall_timer(5s, std::bind(&BottleClassifierNode::update_decision_callback, this));
        
        // Continuously broadcast the current state at 10Hz
        broadcast_timer_ = this->create_wall_timer(100ms, std::bind(&BottleClassifierNode::broadcast_callback, this));
    }

private:
    void load_model(const std::string& model_path, const std::string& scaling_path) {
        try {
            svm_ = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
            cv::FileStorage fs(scaling_path, cv::FileStorage::READ);
            fs["mean"] >> mean_;
            fs["stddev"] >> stddev_;
            fs.release();
            RCLCPP_INFO(this->get_logger(), "Model and scaling parameters loaded.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Init error: %s", e.what());
        }
    }

    void feature_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 5) {
            cv::Mat input_row(1, 5, CV_32F, const_cast<float*>(msg->data.data()));
            cv::Mat scaled_input = cv::Mat::zeros(1, 5, CV_32F);
            for (int i = 0; i < 5; ++i) {
                scaled_input.at<float>(0, i) = (input_row.at<float>(0, i) - mean_.at<float>(0, i)) / stddev_.at<float>(0, i);
            }
            // Add new prediction to buffer for the current 5s window
            predictions_buffer_.push_back(static_cast<int>(svm_->predict(scaled_input)));
        }
    }

    void update_decision_callback() {
        if (!predictions_buffer_.empty()) {
            float sum = std::accumulate(predictions_buffer_.begin(), predictions_buffer_.end(), 0.0f);
            float average = sum / predictions_buffer_.size();
            
            // Update the state that the broadcast timer uses
            last_decision_ = (average >= 0.5f);
            
            RCLCPP_INFO(this->get_logger(), "Decision Updated: %s (Avg: %.2f over %zu samples)", 
                        last_decision_ ? "RECYCLABLE" : "NON-RECYCLABLE", average, predictions_buffer_.size());
            
            predictions_buffer_.clear();
        } else {
            RCLCPP_WARN(this->get_logger(), "No features received in last 5s. Keeping previous decision.");
        }
    }

    void broadcast_callback() {
        // Always publish the current state
        auto msg = std_msgs::msg::Bool();
        msg.data = last_decision_;
        recyclable_pub_->publish(msg);
    }

    // ROS Members
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recyclable_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;

    // Data Members
    cv::Ptr<cv::ml::SVM> svm_;
    cv::Mat mean_;
    cv::Mat stddev_;
    std::vector<int> predictions_buffer_;
    bool last_decision_ = false; // Default state
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleClassifierNode>());
    rclcpp::shutdown();
    return 0;
}