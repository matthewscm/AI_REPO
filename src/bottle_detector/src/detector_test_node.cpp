#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

// ── Feature Extractor Constants & Helpers ──────────────────────────────────────

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

        if ((x2 - x1) < 10 || (y2 - y1) < 10) {
            return img; 
        }
        return img(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
    }

    cv::Vec3f dominant_hsv(const cv::Mat& img_bgr) {
        cv::Mat img_hsv;
        cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsv_channels;
        cv::split(img_hsv, hsv_channels);
        cv::Mat s_ch = hsv_channels[1];

        cv::Mat colour_mask;
        cv::compare(s_ch, MIN_SATURATION, colour_mask, cv::CMP_GE);

        int n_colour = cv::countNonZero(colour_mask);
        int n_total = img_hsv.rows * img_hsv.cols;
        int min_required = std::max(1, static_cast<int>(n_total * MIN_COLOUR_PIXEL_RATIO));

        if (n_colour < min_required) {
            cv::Scalar mean_val = cv::mean(img_hsv);
            return cv::Vec3f(static_cast<float>(mean_val[0]), 
                             static_cast<float>(mean_val[1]), 
                             static_cast<float>(mean_val[2]));
        }

        int channels[] = {0, 1, 2};
        int histSize[] = {H_BINS, S_BINS, V_BINS};
        float h_ranges[] = {0, 180.0f};
        float s_ranges[] = {0, 256.0f};
        float v_ranges[] = {0, 256.0f};
        const float* ranges[] = {h_ranges, s_ranges, v_ranges};

        cv::Mat hist;
        cv::calcHist(&img_hsv, 1, channels, colour_mask, hist, 3, histSize, ranges);

        int maxIdx[3] = {0, 0, 0};
        double maxVal = 0.0;
        cv::minMaxIdx(hist, nullptr, &maxVal, nullptr, maxIdx);

        float dom_h = h_ranges[0] + (maxIdx[0] + 0.5f) * ((h_ranges[1] - h_ranges[0]) / H_BINS);
        float dom_s = s_ranges[0] + (maxIdx[1] + 0.5f) * ((s_ranges[1] - s_ranges[0]) / S_BINS);
        float dom_v = v_ranges[0] + (maxIdx[2] + 0.5f) * ((v_ranges[1] - v_ranges[0]) / V_BINS);

        return cv::Vec3f(dom_h, dom_s, dom_v);
    }

    std::vector<float> extract(const cv::Mat& image) {
        if (image.empty()) return {0, 0, 0, 0, 0};
        float width = static_cast<float>(image.cols);
        float height = static_cast<float>(image.rows);
        cv::Mat cropped = centre_crop(image);
        cv::Vec3f hsv = dominant_hsv(cropped);
        return {width, height, hsv[0], hsv[1], hsv[2]};
    }
}

// ── ROS 2 Node ─────────────────────────────────────────────────────────────────

class BottleDetectorTestNode : public rclcpp::Node {
public:
    BottleDetectorTestNode() : Node("bottle_detector_test") {
        RCLCPP_INFO(this->get_logger(), "Starting Static Image Bottle Test...");

        // 1. Resolve Paths
        std::string pkg_share = ament_index_cpp::get_package_share_directory("bottle_detector");
        std::string model_path = pkg_share + "/models/yolov8s.onnx";
        std::string image_path = "/home/connor/ros2_ws/src/bottle_detector/images/Assorted_bottles.jpeg";

        // 2. Load the Image
        cv::Mat frame = cv::imread(image_path);
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
            return;
        }

        // 3. Load the Neural Network
        cv::dnn::Net net = cv::dnn::readNet(model_path);
        if (net.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model!");
            return;
        }

        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // 4. Pre-process (Strictly 640x640)
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net.setInput(blob);

        // 5. Run Inference
        RCLCPP_INFO(this->get_logger(), "Starting Inference...");
        std::vector<cv::Mat> outputs;
        net.forward(outputs); 

        // 6. Diagnostic Print & Safety Check
        if (outputs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Network produced no output!");
            return;
        }

        cv::Mat output = outputs[0];
        RCLCPP_INFO(this->get_logger(), "SUCCESS! Inference completed.");
        
        // 7. Universal Post-Processing
        cv::Mat data;
        if (output.dims == 3) {
            int dim1 = output.size[1]; 
            int dim2 = output.size[2]; 
            
            if (dim1 == 84 || dim1 == 85) {
                data = cv::Mat(dim1, dim2, CV_32F, output.ptr<float>());
                data = data.t(); 
            } else {
                data = cv::Mat(dim1, dim2, CV_32F, output.ptr<float>());
            }
        } else if (output.dims == 2) {
            data = output;
            if (data.rows == 84 || data.rows == 85) {
                data = data.t();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unexpected output dimensions from ONNX!");
            return;
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
                if (obj_conf > 0.5) { 
                    bottle_score = obj_conf * data.at<float>(i, class_offset + 39);
                }
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
                int width  = static_cast<int>(w * x_scale);
                int height = static_cast<int>(h * y_scale);

                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(bottle_score);
            }
        }

        // 8. Non-Maximum Suppression (NMS)
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        RCLCPP_INFO(this->get_logger(), "Detected %zu bottles after NMS.", indices.size());

        // 9. Extract Features, Draw Boxes and Save
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cv::Rect box = boxes[idx];

            // Safety boundary check before cropping
            cv::Rect frame_bounds(0, 0, frame.cols, frame.rows);
            cv::Rect safe_box = box & frame_bounds;

            if (safe_box.width > 0 && safe_box.height > 0) {
                // Extract features for this specific bottle
                cv::Mat bottle_crop = frame(safe_box);
                std::vector<float> features = feature_extractor::extract(bottle_crop);
                
                // Log the results directly to the console
                RCLCPP_INFO(this->get_logger(), 
                    "Bottle %zu: Width=%.0fpx, Height=%.0fpx | HSV=(%.1f, %.1f, %.1f)", 
                    i + 1, features[0], features[1], features[2], features[3], features[4]);
            }

            // Draw bounding boxes (using original box to keep visual proportions)
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 3); 
            
            std::string label = "Bottle: " + std::to_string(static_cast<int>(confidences[idx] * 100)) + "%";
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
            cv::rectangle(frame, cv::Point(box.x, box.y - labelSize.height - baseLine), 
                          cv::Point(box.x + labelSize.width, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, box.y - baseLine), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        }

        // Fixed string to avoid overwriting original source image if in same directory
        cv::imwrite("RGB_bottles_results.png", frame);
        RCLCPP_INFO(this->get_logger(), "Saved output to 'RGB_bottles_results.png'");
        
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleDetectorTestNode>();
    rclcpp::spin(node);
    return 0;
}