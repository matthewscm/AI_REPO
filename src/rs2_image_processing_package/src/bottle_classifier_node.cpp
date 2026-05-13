#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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

class BottleClassifierNode : public rclcpp::Node
{
public:
    BottleClassifierNode() : Node("bottle_classifier_node"), idle_(true), readings_count_(0), sum_x_(0.0), sum_y_(0.0), sum_z_(0.0), intrinsics_received_(false), fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0)
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

        rclcpp::QoS narrow_qos(10); // Only keep the very latest frame
        narrow_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // 3. Initialize subscriptions for Depth, RGB, and Camera Info topics
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/aligned_depth_to_color/image_raw", narrow_qos,
            std::bind(&BottleClassifierNode::depth_callback, this, std::placeholders::_1));

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", qos,
            std::bind(&BottleClassifierNode::image_callback, this, std::placeholders::_1));

        // Uncommented and initialized camera_info subscription
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera/aligned_depth_to_color/camera_info", 10,
            std::bind(&BottleClassifierNode::camera_info_callback, this, std::placeholders::_1));

        sys_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "system_command", 10, 
            std::bind(&BottleClassifierNode::sys_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Bottle Classifier Started. Idling until '4' is received on 'system_command' topic.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sys_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr category_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr class_id_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr avg_center_pub_;

    cv::Mat latest_depth_img_;
    std::mutex depth_mutex_;

    bool idle_; // State flag to control when to process a frame
    int readings_count_; // Counter for averaging
    double sum_x_;
    double sum_y_;
    double sum_z_;

    // Camera intrinsics
    bool intrinsics_received_;
    double fx_, fy_, cx_, cy_;

    // Callback to extract camera intrinsics
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        if (!intrinsics_received_) {
            // Intrinsic camera matrix for the raw (distorted) images.
            //     [fx  0 cx]
            // K = [ 0 fy cy]
            //     [ 0  0  1]
            fx_ = msg->k[0];
            cx_ = msg->k[2];
            fy_ = msg->k[4];
            cy_ = msg->k[5];
            intrinsics_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        }
    }

    // Callback to trigger processing when '4' is received
    void sys_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg)
    {
        if (msg->data == 4) {
            if (idle_) {
                RCLCPP_INFO(this->get_logger(), "Received command 4. Collecting 10 frames for average...");
                readings_count_ = 0;
                sum_x_ = 0.0;
                sum_y_ = 0.0;
                sum_z_ = 0.0;
                idle_ = false; // Wake up
            } else {
                RCLCPP_WARN(this->get_logger(), "Received command 4, but already processing frames. Ignoring.");
            }
        }
    }

    // Callback to store the latest aligned depth image
    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv::Mat depth_img = cv_bridge::toCvShare(msg, "16UC1")->image;
            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_img_ = depth_img.clone();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
        }
    }

    // Main callback executed whenever an RGB image arrives
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (idle_) {
            return;
        }

        try
        {
            cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat hsv_img;
            cv::cvtColor(color_img, hsv_img, cv::COLOR_BGR2HSV);

            cv::Mat mask_red1, mask_red2, mask_red, mask_green, mask_blue;

            cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red1);
            cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask_red2);
            cv::bitwise_or(mask_red1, mask_red2, mask_red); 

            cv::inRange(hsv_img, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), mask_green);
            cv::inRange(hsv_img, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), mask_blue);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel);

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
            int class_id = -1;
            cv::Rect best_bbox;
            cv::Scalar box_color;

            double min_area_threshold = 1500.0; 

            if (red_info.first > min_area_threshold) {
                category = "Red Cap Bottle";
                class_id = 0;
                best_bbox = red_info.second;
                box_color = cv::Scalar(0, 0, 255); 
            } else if (green_info.first > min_area_threshold) {
                category = "Green Cap Bottle";
                class_id = 1;
                best_bbox = green_info.second;
                box_color = cv::Scalar(0, 255, 0); 
            } else if (blue_info.first > min_area_threshold) {
                category = "Blue Cap Bottle";
                class_id = 2;
                best_bbox = blue_info.second;
                box_color = cv::Scalar(255, 0, 0); 
            }

            cv::Mat display_img = color_img.clone();
            int center_x = -1;
            int center_y = -1;
            float depth_z = 0.0;
            
            if (class_id != -1) {
                center_x = best_bbox.x + best_bbox.width / 2;
                center_y = best_bbox.y + best_bbox.height / 2;

                {
                    std::lock_guard<std::mutex> lock(depth_mutex_);
                    if (!latest_depth_img_.empty() && 
                        center_x >= 0 && center_x < latest_depth_img_.cols && 
                        center_y >= 0 && center_y < latest_depth_img_.rows) {
                        
                        uint16_t depth_mm = latest_depth_img_.at<uint16_t>(center_y, center_x);
                        depth_z = static_cast<float>(depth_mm) / 1000.0f; // Convert mm to meters
                    }
                }

                // --- Calculate 3D coordinates in meters ---
                double real_x = 0.0;
                double real_y = 0.0;

                if (intrinsics_received_ && depth_z > 0.0) {
                    real_x = (center_x - cx_) * depth_z / fx_;
                    real_y = (center_y - cy_) * depth_z / fy_;
                } else if (!intrinsics_received_) {
                    RCLCPP_WARN(this->get_logger(), "Camera intrinsics not received yet. 3D coordinates will be 0.0.");
                }

                // Drawing on image (still requires pixel coordinates)
                cv::rectangle(display_img, best_bbox, box_color, 2);
                cv::circle(display_img, cv::Point(center_x, center_y), 5, box_color, -1); 
                
                std::stringstream depth_ss;
                depth_ss << std::fixed << std::setprecision(3) << depth_z;
                std::string label = category + " (" + depth_ss.str() + "m)";
                
                cv::putText(display_img, label, cv::Point(best_bbox.x, std::max(best_bbox.y - 10, 0)), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.9, box_color, 2);

                // Publish the 3D center point in METERS
                geometry_msgs::msg::Point center_msg;
                center_msg.x = real_x;
                center_msg.y = real_y;
                center_msg.z = depth_z;
                center_pub_->publish(center_msg);

                // Accumulate meter readings for average
                sum_x_ += real_x;
                sum_y_ += real_y;
                sum_z_ += depth_z;
                readings_count_++;

                RCLCPP_INFO(this->get_logger(),
                            "Detected [%d/10]: [%s] | Pixel: (%d, %d) | 3D: (%.3f, %.3f, %.3fm)", 
                            readings_count_, category.c_str(), center_x, center_y, real_x, real_y, depth_z);
                
                if (readings_count_ >= 10) {
                    geometry_msgs::msg::Point avg_msg;
                    avg_msg.x = sum_x_ / 10.0;
                    avg_msg.y = sum_y_ / 10.0;
                    avg_msg.z = sum_z_ / 10.0;
                    avg_center_pub_->publish(avg_msg);

                    RCLCPP_INFO(this->get_logger(), "Collected 10 readings. Average published (meters): (%.3f, %.3f, %.3f). Returning to IDLE.", avg_msg.x, avg_msg.y, avg_msg.z);
                    idle_ = true; 
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Triggered, but no bottle detected in the frame.");
            }

            std_msgs::msg::String str_msg;
            str_msg.data = category;
            category_pub_->publish(str_msg);

            std_msgs::msg::Int32 id_msg;
            id_msg.data = class_id;
            class_id_pub_->publish(id_msg);

            sensor_msgs::msg::Image::SharedPtr out_img_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", display_img).toImageMsg();
            image_pub_->publish(*out_img_msg);

        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            idle_ = true; 
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
