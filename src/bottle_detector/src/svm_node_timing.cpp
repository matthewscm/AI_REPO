#include <memory>
#include <vector>
#include <string>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class BottleClassifierNode : public rclcpp::Node {
public:
    BottleClassifierNode() : Node("bottle_classifier_node") {
        // 1. Setup paths
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string model_path = pkg_share + "/models/bottle_svm_model.xml";
        std::string scaling_path = pkg_share + "/models/scaling_params.xml";

        // 2. Load SVM and Scaling Params
        try {
            svm_ = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
            if (!svm_) throw std::runtime_error("Failed to load SVM model");

            cv::FileStorage fs(scaling_path, cv::FileStorage::READ);
            if (!fs.isOpened()) throw std::runtime_error("Failed to load scaling params");
            fs["mean"] >> mean_;
            fs["stddev"] >> stddev_;
            fs.release();
            
            RCLCPP_INFO(this->get_logger(), "Model and scaling parameters loaded.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization error: %s", e.what());
        }

        // 3. Pub/Sub
        feature_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bottle_features", 10, std::bind(&BottleClassifierNode::feature_callback, this, std::placeholders::_1));

        recyclable_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_recyclable", 10);

        // 4. Timer: Triggers every 5 seconds to process the accumulated data
        timer_ = this->create_wall_timer(5s, std::bind(&BottleClassifierNode::timer_callback, this));
    }

private:
    void feature_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 5) return;

        // Convert and Scale
        cv::Mat input_row(1, 5, CV_32F, const_cast<float*>(msg->data.data()));
        cv::Mat scaled_input = cv::Mat::zeros(1, 5, CV_32F);
        for (int i = 0; i < 5; ++i) {
            scaled_input.at<float>(0, i) = (input_row.at<float>(0, i) - mean_.at<float>(0, i)) / stddev_.at<float>(0, i);
        }

        // Predict and store result in the buffer
        float prediction = svm_->predict(scaled_input);
        predictions_buffer_.push_back(static_cast<int>(prediction));
    }

    void timer_callback() {
        if (predictions_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data received in the last 5 seconds.");
            return;
        }

        // Calculate the average/majority
        // Since it's binary (1 or 0), the sum divided by size gives the ratio
        float sum = std::accumulate(predictions_buffer_.begin(), predictions_buffer_.end(), 0.0f);
        float average = sum / predictions_buffer_.size();

        // If average > 0.5, majority voted for Recyclable (1)
        bool final_decision = (average >= 0.5f);

        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = final_decision;
        recyclable_pub_->publish(bool_msg);

        RCLCPP_INFO(this->get_logger(), "Window Closed. Samples: %zu | Avg: %.2f | Decision: %s", 
                    predictions_buffer_.size(), average, (final_decision ? "Recyclable" : "Non-Recyclable"));

        // Clear buffer for the next 5-second window
        predictions_buffer_.clear();
    }

    // ROS members
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recyclable_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data members
    cv::Ptr<cv::ml::SVM> svm_;
    cv::Mat mean_;
    cv::Mat stddev_;
    std::vector<int> predictions_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleClassifierNode>());
    rclcpp::shutdown();
    return 0;
}