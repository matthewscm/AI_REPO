#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BottleClassifierNode : public rclcpp::Node
{
public:
    BottleClassifierNode() : Node("bottle_classifier_node")
    {
        // 1. Initialize QoS settings for sensor data
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // 2. Initialize publisher for the result
        category_pub_ = this->create_publisher<std_msgs::msg::String>("bottle_category", 10);

        // 3. Initialize a standard subscription to the RGB topic (No synchronizer needed!)
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", qos,
            std::bind(&BottleClassifierNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RGB-only Categorizer Node started. Waiting for images...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr category_pub_;

    // Main callback executed whenever an RGB image arrives
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // --- STEP 1: Convert ROS message to OpenCV format ---
            cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;

            // --- STEP 2: Convert directly to HSV Color Space ---
            // We skip depth masking entirely and just process the whole image
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

            // --- STEP 4: Count Pixels and Categorize ---
            int red_pixels = cv::countNonZero(mask_red);
            int green_pixels = cv::countNonZero(mask_green);
            int blue_pixels = cv::countNonZero(mask_blue);

            std::string category = "Unknown";
            int max_pixels = std::max({red_pixels, green_pixels, blue_pixels});

            // Threshold: must see at least 500 pixels of the color to count it
            int min_pixel_threshold = 500; 

            if (max_pixels > min_pixel_threshold) {
                if (max_pixels == red_pixels) {
                    category = "Red Label Bottle";
                } else if (max_pixels == green_pixels) {
                    category = "Green Label Bottle";
                } else if (max_pixels == blue_pixels) {
                    category = "Blue Label Bottle";
                }
            }

            // --- STEP 5: Publish the result ---
            std_msgs::msg::String out_msg;
            out_msg.data = category;
            category_pub_->publish(out_msg);

            // Log output to terminal
            RCLCPP_INFO(this->get_logger(), "Detected: %s (R:%d, G:%d, B:%d)", 
                        category.c_str(), red_pixels, green_pixels, blue_pixels);

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


