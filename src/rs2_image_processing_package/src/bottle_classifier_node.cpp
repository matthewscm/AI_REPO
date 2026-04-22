#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <utility>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>

class BottleClassifierNode : public rclcpp::Node
{
public:
    BottleClassifierNode() : Node("bottle_classifier_node")
    {
        // 1. Initialize QoS settings for sensor data
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // 2. Initialize publishers for the results
        category_pub_ = this->create_publisher<std_msgs::msg::String>("bottle_category", 10);
        class_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("bottle_class_id", 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("bottle_center", 10);
        avg_center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("bottle_center_average", 10);

        // 3. Initialize subscriptions for Depth and RGB topics
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/aligned_depth_to_color/image_raw", qos,
            std::bind(&BottleClassifierNode::depth_callback, this, std::placeholders::_1));

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", qos,
            std::bind(&BottleClassifierNode::image_callback, this, std::placeholders::_1));

        // 4. Set a timer to shut down the node after 5 seconds and publish averages
        shutdown_timer_ = this->create_wall_timer(
            std::chrono::seconds(20),
            [this]() {
                if (detection_count_ > 0) {
                    double avg_x = sum_x_ / detection_count_;
                    double avg_y = sum_y_ / detection_count_;
                    double avg_z = sum_z_ / detection_count_;
                    
                    RCLCPP_INFO(this->get_logger(), "=========================================");
                    RCLCPP_INFO(this->get_logger(), "FINAL AVERAGE CENTER: (%.2f, %.2f, %.3fm)", avg_x, avg_y, avg_z);
                    RCLCPP_INFO(this->get_logger(), "Total Valid Detections: %d", detection_count_);
                    RCLCPP_INFO(this->get_logger(), "=========================================");

                    geometry_msgs::msg::Point avg_msg;
                    avg_msg.x = avg_x;
                    avg_msg.y = avg_y;
                    avg_msg.z = avg_z;
                    avg_center_pub_->publish(avg_msg);
                    
                    // Small delay to ensure the final message is sent before the node exits
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } else {
                    RCLCPP_INFO(this->get_logger(), "No bottles detected during the 5 seconds. No average to publish.");
                }

                RCLCPP_INFO(this->get_logger(), "5 seconds have elapsed. Auto-shutting down node.");
                rclcpp::shutdown();
            });

        RCLCPP_INFO(this->get_logger(), "RGB-D Multi-Categorizer Node started. Will automatically shut down in 5 seconds...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr category_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr class_id_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr avg_center_pub_;

    cv::Mat latest_depth_img_;
    std::mutex depth_mutex_;

    // Variables to accumulate values for the final average
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;
    double sum_z_ = 0.0;
    int detection_count_ = 0;

    // Callback to store the latest aligned depth image
    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            // RealSense depth is typically 16-bit unsigned int representing millimeters
            cv::Mat depth_img = cv_bridge::toCvShare(msg, "16UC1")->image;
            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_img_ = depth_img.clone(); // Keep safely in memory
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
        }
    }

    // Main callback executed whenever an RGB image arrives
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // --- STEP 1: Convert ROS message to OpenCV format ---
            cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;

            // --- STEP 2: Convert directly to HSV Color Space ---
            cv::Mat hsv_img;
            cv::cvtColor(color_img, hsv_img, cv::COLOR_BGR2HSV);

            // --- STEP 3: Define Color Ranges ---
            cv::Mat mask_red1, mask_red2, mask_red, mask_green, mask_blue;

            // Red (requires two ranges due to wrapping around the HSV cylinder)
            cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red1);
            cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask_red2);
            cv::bitwise_or(mask_red1, mask_red2, mask_red); 

            // Green and Blue
            cv::inRange(hsv_img, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), mask_green);
            cv::inRange(hsv_img, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), mask_blue);

            // --- STEP 4: Clean up masks and find largest contours ---
            // Remove scattered background noise using morphological opening
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel);

            // Helper to find the largest contiguous blob of color AND its bounding box
            auto getLargestContourInfo = [](const cv::Mat& mask) -> std::pair<double, cv::Rect> {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                double max_area = 0.0;
                cv::Rect bounding_box;
                for (const auto& contour : contours) {
                    double area = cv::contourArea(contour);
                    if (area > max_area) {
                        max_area = area;
                        bounding_box = cv::boundingRect(contour);
                    }
                }
                return {max_area, bounding_box};
            };

            auto red_info = getLargestContourInfo(mask_red);
            auto green_info = getLargestContourInfo(mask_green);
            auto blue_info = getLargestContourInfo(mask_blue);

            std::string category = "Unknown";
            int class_id = -1; // -1 represents nothing detected
            cv::Rect best_bbox;
            cv::Scalar box_color;

            // Threshold: must see a contiguous blob of at least 1500 pixels
            double min_area_threshold = 1500.0; 

            // Hierarchical check: Red [0] -> Green [1] -> Blue [2]
            if (red_info.first > min_area_threshold) {
                category = "Red Label Bottle";
                class_id = 0;
                best_bbox = red_info.second;
                box_color = cv::Scalar(0, 0, 255); // BGR Red
            } else if (green_info.first > min_area_threshold) {
                category = "Green Label Bottle";
                class_id = 1;
                best_bbox = green_info.second;
                box_color = cv::Scalar(0, 255, 0); // BGR Green
            } else if (blue_info.first > min_area_threshold) {
                category = "Blue Label Bottle";
                class_id = 2;
                best_bbox = blue_info.second;
                box_color = cv::Scalar(255, 0, 0); // BGR Blue
            }

            // --- STEP 5: Draw Bounding Box and Publish Results ---
            cv::Mat display_img = color_img.clone();
            int center_x = -1;
            int center_y = -1;
            float depth_z = 0.0;
            
            if (class_id != -1) {
                // Calculate center point
                center_x = best_bbox.x + best_bbox.width / 2;
                center_y = best_bbox.y + best_bbox.height / 2;

                // Extract Depth at the center coordinate
                {
                    std::lock_guard<std::mutex> lock(depth_mutex_);
                    if (!latest_depth_img_.empty() && 
                        center_x >= 0 && center_x < latest_depth_img_.cols && 
                        center_y >= 0 && center_y < latest_depth_img_.rows) {
                        
                        uint16_t depth_mm = latest_depth_img_.at<uint16_t>(center_y, center_x);
                        depth_z = static_cast<float>(depth_mm) / 1000.0f; // Convert mm to meters
                    }
                }

                // Draw the rectangle, center dot, and category text on the cloned image
                cv::rectangle(display_img, best_bbox, box_color, 2);
                cv::circle(display_img, cv::Point(center_x, center_y), 5, box_color, -1); // Solid circle at center
                
                // Add depth text to visualization with 3 decimal precision (mm level)
                std::stringstream depth_ss;
                depth_ss << std::fixed << std::setprecision(3) << depth_z;
                std::string label = category + " (" + depth_ss.str() + "m)";
                
                cv::putText(display_img, label, cv::Point(best_bbox.x, std::max(best_bbox.y - 10, 0)), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2);

                // Publish the 3D center point
                geometry_msgs::msg::Point center_msg;
                center_msg.x = center_x;
                center_msg.y = center_y;
                center_msg.z = depth_z;
                center_pub_->publish(center_msg);

                // Accumulate data for the final average
                sum_x_ += center_x;
                sum_y_ += center_y;
                sum_z_ += depth_z;
                detection_count_++;
            }

            // Publish the string category
            std_msgs::msg::String str_msg;
            str_msg.data = category;
            category_pub_->publish(str_msg);

            // Publish the integer class ID
            std_msgs::msg::Int32 id_msg;
            id_msg.data = class_id;
            class_id_pub_->publish(id_msg);

            // Publish the processed image
            sensor_msgs::msg::Image::SharedPtr out_img_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", display_img).toImageMsg();
            image_pub_->publish(*out_img_msg);

            // Log output to terminal ONLY every 250 ms (0.25 seconds)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, 
                        "Detected: [%s] | ID: %d | Center: (%d, %d, %.3fm) | (R Area:%.0f, G Area:%.0f, B Area:%.0f)", 
                        category.c_str(), class_id, center_x, center_y, depth_z, red_info.first, green_info.first, blue_info.first);

        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleClassifierNode>());
    rclcpp::shutdown();
    return 0;
}
