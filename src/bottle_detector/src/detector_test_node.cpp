#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

// ROS 2 Message Headers
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// ── Feature Extractor (Matches Python Logic) ──────────────────────────────────

namespace feature_extractor {
    const int H_BINS = 36;
    const int S_BINS = 32;
    const int V_BINS = 32;
    const int MIN_SATURATION = 30;
    const float MIN_COLOUR_PIXEL_RATIO = 0.05f;
    const float LABEL_X_MARGIN = 0.20f; 
    const float LABEL_Y_MARGIN = 0.25f; 

    cv::Mat centre_crop(const cv::Mat& img) {
        int h = img.rows;
        int w = img.cols;
        int x1 = static_cast<int>(w * LABEL_X_MARGIN);
        int x2 = static_cast<int>(w * (1.0f - LABEL_X_MARGIN));
        int y1 = static_cast<int>(h * LABEL_Y_MARGIN);
        int y2 = static_cast<int>(h * (1.0f - LABEL_Y_MARGIN));

        if ((x2 - x1) < 10 || (y2 - y1) < 10) return img; 
        return img(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
    }

    cv::Vec3f dominant_hsv(const cv::Mat& img_bgr) {
        cv::Mat img_hsv;
        cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsv_channels;
        cv::split(img_hsv, hsv_channels);
        cv::Mat s_ch = hsv_channels[1];
        cv::Mat mask;
        cv::compare(s_ch, MIN_SATURATION, mask, cv::CMP_GE);

        int n_colour = cv::countNonZero(mask);
        int n_total = img_hsv.rows * img_hsv.cols;
        if (n_colour < std::max(1, static_cast<int>(n_total * MIN_COLOUR_PIXEL_RATIO))) {
            cv::Scalar mean_val = cv::mean(img_hsv);
            return cv::Vec3f(mean_val[0], mean_val[1], mean_val[2]);
        }

        int channels[] = {0, 1, 2};
        int histSize[] = {H_BINS, S_BINS, V_BINS};
        float h_ranges[] = {0, 180};
        float s_ranges[] = {0, 256};
        float v_ranges[] = {0, 256};
        const float* ranges[] = {h_ranges, s_ranges, v_ranges};

        cv::Mat hist;
        cv::calcHist(&img_hsv, 1, channels, mask, hist, 3, histSize, ranges);

        int maxIdx[3] = {0, 0, 0};
        cv::minMaxIdx(hist, nullptr, nullptr, nullptr, maxIdx);

        float dom_h = (maxIdx[0] + 0.5f) * (180.0f / H_BINS);
        float dom_s = (maxIdx[1] + 0.5f) * (256.0f / S_BINS);
        float dom_v = (maxIdx[2] + 0.5f) * (256.0f / V_BINS);

        return cv::Vec3f(dom_h, dom_s, dom_v);
    }

    std::vector<float> extract(const cv::Mat& cropped_bottle_image) {
        if (cropped_bottle_image.empty()) return {0, 0, 0, 0, 0, 0};
        
        float width = static_cast<float>(cropped_bottle_image.cols);
        float height = static_cast<float>(cropped_bottle_image.rows);
        // float aspect_ratio = width / height;
        
        cv::Mat focused_region = centre_crop(cropped_bottle_image);
        cv::Vec3f hsv = dominant_hsv(focused_region);
        
        return {width, height, hsv[0], hsv[1], hsv[2]};
    }
}

// ── Continuous Live ROS 2 Node ────────────────────────────────────────────────

class BottleDetectorLiveNode : public rclcpp::Node {
public:
    BottleDetectorLiveNode() : Node("bottle_detector_live") {
        
        // 1. Setup Publishers and Subscribers
        feature_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("bottle_features", 10);
        
        // Subscribe to RealSense color topic (update this string if your topic name is different)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&BottleDetectorLiveNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Initializing Live Bottle Detector Node...");

        // 2. Load the Neural Network ONCE in the constructor
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string model_path = pkg_share + "/models/yolov8s.onnx";

        net_ = cv::dnn::readNet(model_path);
        if (net_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model from %s", model_path.c_str());
            return;
        }

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        
        RCLCPP_INFO(this->get_logger(), "Model loaded. Waiting for RealSense camera feed...");
    }

private:
    cv::dnn::Net net_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr feature_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // This callback runs every time a new frame arrives from the camera
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        
        // Convert ROS image message to OpenCV Mat
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        // 3. Pre-process and Inference
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        net_.forward(outputs); 

        if (outputs.empty()) return;

        // 4. Universal Post-Processing
        cv::Mat output = outputs[0];
        cv::Mat data;
        
        if (output.dims == 3) {
            int dim1 = output.size[1]; 
            int dim2 = output.size[2]; 
            if (dim1 == 84 || dim1 == 85) {
                data = cv::Mat(dim1, dim2, CV_32F, output.ptr<float>()).t(); 
            } else {
                data = cv::Mat(dim1, dim2, CV_32F, output.ptr<float>());
            }
        } else if (output.dims == 2) {
            data = output;
            if (data.rows == 84 || data.rows == 85) data = data.t();
        }

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        float x_scale = frame.cols / 640.0f;
        float y_scale = frame.rows / 640.0f;

        bool has_objectness = (data.cols == 85);
        int class_offset = has_objectness ? 5 : 4; 

        for (int i = 0; i < data.rows; ++i) {
            float bottle_score = 0.0f;
            if (has_objectness) {
                float obj_conf = data.at<float>(i, 4);
                if (obj_conf > 0.5) bottle_score = obj_conf * data.at<float>(i, class_offset + 39); // 39 is COCO 'bottle'
            } else {
                bottle_score = data.at<float>(i, class_offset + 39);
            }

            if (bottle_score > 0.5) { 
                float cx = data.at<float>(i, 0);
                float cy = data.at<float>(i, 1);
                float w  = data.at<float>(i, 2);
                float h  = data.at<float>(i, 3);

                int left   = static_cast<int>((cx - w / 2) * x_scale);
                int top    = static_cast<int>((cy - h / 2) * y_scale);
                boxes.push_back(cv::Rect(left, top, static_cast<int>(w * x_scale), static_cast<int>(h * y_scale)));
                confidences.push_back(bottle_score);
            }
        }

        // 5. NMS & Feature Extraction
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        auto feature_msg = std_msgs::msg::Float32MultiArray();

        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cv::Rect safe_box = boxes[idx] & cv::Rect(0, 0, frame.cols, frame.rows);

            if (safe_box.width > 0 && safe_box.height > 0) {
                cv::Mat bottle_crop = frame(safe_box);
                std::vector<float> features = feature_extractor::extract(bottle_crop);

                feature_msg.data.insert(feature_msg.data.end(), features.begin(), features.end());
                
                // Draw live bounding boxes
                cv::rectangle(frame, safe_box, cv::Scalar(0, 255, 0), 3); 
                std::string label = "Bottle: " + std::to_string(static_cast<int>(confidences[idx] * 100)) + "%";
                cv::putText(frame, label, cv::Point(safe_box.x, safe_box.y - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            }
        }

        // 6. Publish Features
        if (!feature_msg.data.empty()) {
            feature_pub_->publish(feature_msg);
        }

        // 7. Show live feed via OpenCV Window
        cv::imshow("Live Bottle Detections", frame);
        cv::waitKey(1); // Required to refresh the UI window
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleDetectorLiveNode>();
    rclcpp::spin(node); // This now loops continuously
    rclcpp::shutdown();
    return 0;
}