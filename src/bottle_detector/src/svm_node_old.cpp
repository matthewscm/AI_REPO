#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/bool.hpp" // Added Boolean message header

class BottleClassifierNode : public rclcpp::Node {
public:
    BottleClassifierNode() : Node("bottle_classifier_node") {
        // 1. Setup paths
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string model_path = pkg_share + "/models/bottle_svm_model.xml";
        std::string scaling_path = pkg_share + "/models/scaling_params.xml";

        // 2. Load the SVM model
        try {
            svm_ = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
            if (!svm_) throw std::runtime_error("Failed to load SVM model");

            // 3. Load scaling parameters (mean and stddev)
            cv::FileStorage fs(scaling_path, cv::FileStorage::READ);
            if (!fs.isOpened()) throw std::runtime_error("Failed to load scaling params");
            fs["mean"] >> mean_;
            fs["stddev"] >> stddev_;
            fs.release();
            
            RCLCPP_INFO(this->get_logger(), "Model and scaling parameters loaded successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Initialization error: %s", e.what());
        }

        // 4. Initialize Pub/Sub
        // Subscribes to the features array
        feature_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bottle_features", 10, std::bind(&BottleClassifierNode::feature_callback, this, std::placeholders::_1));

        // Publishes the classification result (1 for recyclable, 0 for not)
        result_pub_ = this->create_publisher<std_msgs::msg::Int32>("classification_result", 10);

        // New Boolean publisher
        recyclable_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_recyclable", 10);
    }

private:
    void feature_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 5) {
            RCLCPP_WARN(this->get_logger(), "Expected 5 features, received %zu", msg->data.size());
            return;
        }

        // Convert message to cv::Mat
        cv::Mat input_row(1, 5, CV_32F, const_cast<float*>(msg->data.data()));

        // 5. Apply Feature Scaling (Crucial for SVM performance)
        cv::Mat scaled_input = cv::Mat::zeros(1, 5, CV_32F);
        for (int i = 0; i < 5; ++i) {
            scaled_input.at<float>(0, i) = (input_row.at<float>(0, i) - mean_.at<float>(0, i)) / stddev_.at<float>(0, i);
        }

        RCLCPP_INFO(this->get_logger(), 
        "RAW: [%.1f, %.1f, %.1f, %.1f, %.1f] | SCALED: [%.2f, %.2f, %.2f, %.2f, %.2f]",
        input_row.at<float>(0,0), input_row.at<float>(0,1), input_row.at<float>(0,2), 
        input_row.at<float>(0,3), input_row.at<float>(0,4),
        scaled_input.at<float>(0,0), scaled_input.at<float>(0,1), scaled_input.at<float>(0,2), 
        scaled_input.at<float>(0,3), scaled_input.at<float>(0,4));

        // 6. Predict
        float prediction = svm_->predict(scaled_input);

        // 7. Publish results
        int prediction_int = static_cast<int>(prediction);

        auto result_msg = std_msgs::msg::Int32();
        result_msg.data = prediction_int;
        result_pub_->publish(result_msg);

        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = (prediction_int == 1);
        recyclable_pub_->publish(bool_msg);

        RCLCPP_INFO(this->get_logger(), "Prediction: %s", (bool_msg.data ? "Recyclable" : "Non-Recyclable"));
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recyclable_pub_; // New Publisher member
    
    cv::Ptr<cv::ml::SVM> svm_;
    cv::Mat mean_;
    cv::Mat stddev_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleClassifierNode>());
    rclcpp::shutdown();
    return 0;
}