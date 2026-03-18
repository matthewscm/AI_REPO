#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/bool.hpp> 

class CameraNode : public rclcpp::Node
{
public:
// Constructor

    CameraNode() : Node("camera_node")
    {
        // Publisher for ROS2 camera topic
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // Subscriber for bottle detection flag
        bottle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "bottle_detected", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->bottle_detected_ = msg->data;
                if (msg->data) {
                    RCLCPP_INFO(this->get_logger(), "Bottle detected!");
                }
            }
        );

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/color/image_raw", 10,
        std::bind(&CameraNode::image_callback, this, std::placeholders::_1)
        );
        // Open camera (0= laptop webcam)
        //cap_.open(0);
        // if (!cap_.isOpened()) {
            // RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            // rclcpp::shutdown();
        // }

        // Create OpenCV window
        cv::namedWindow("Live Camera Feed", cv::WINDOW_NORMAL);
        // Size the window smaller (640*0.9 = 576, 480*0.9 = 432)
        cv::resizeWindow("Live Camera Feed", 576, 432);
        // Timer for publishing frames
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(16),  // ~60 FPS
        //     std::bind(&CameraNode::timer_callback, this)
        // );
    }

private:
    // Design text for displaying on the video feed
    void display_text(cv::Mat &frame,
                    const std::string &text,
                    const cv::Point &position = cv::Point(-1, -1), // default bottom-right
                    double font_scale = 1.0,
                    const cv::Scalar &colour = cv::Scalar(0, 255, 0)) // default green
    {
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        int thickness = 2;

        // Get text size
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        baseline += thickness;

        // If position is (-1,-1), put at bottom-right with 10 px margin
        cv::Point text_org = position;
        if (position.x == -1 && position.y == -1)
        {
            text_org = cv::Point(frame.cols - text_size.width - 10, frame.rows - 10);
        }

        // Draw the text
        cv::putText(frame, text, text_org, font_face, font_scale, colour, thickness);
    }

    // Timer callback to capture, display, and publish frames
    // void timer_callback()
    // {
    //     cv::Mat frame;
    //     if (!cap_.read(frame)) {
    //         RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
    //         return;
    //     }

    //     // Default bottom-right green text
    //     display_text(frame, "Live Feed");

    //     // If bottle detected, show top-left red text
    //     if (bottle_detected_) {
    //         // Top-left, larger red text
    //         display_text(frame, "Bottle Detected", cv::Point(10, 30), 1.5, cv::Scalar(0, 0, 255));
    //     }
    //     // --- Show live feed in OpenCV window ---
    //     cv::imshow("Live Camera Feed", frame);
    //     cv::waitKey(1);

    //     // --- Publish to ROS2 topic ---
    //     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    //     pub_->publish(*msg);
    // }

    //Image Callback 

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        display_text(frame, "Live Feed");

        if (bottle_detected_) {
            display_text(frame, "Bottle Detected", cv::Point(10, 30), 1.5, cv::Scalar(0, 0, 255));
        }

        // --- Show live feed ---
        cv::imshow("Live Camera Feed", frame);
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC to quit
            rclcpp::shutdown();
        }

        // --- Publish to ROS2 topic ---
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*out_msg);

        RCLCPP_INFO(this->get_logger(), "Receiving image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bottle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // For laptop Camera 
    //cv::VideoCapture cap_;
    //rclcpp::TimerBase::SharedPtr timer_;
    bool bottle_detected_ = false;
};



// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    // Close OpenCV windows on exit
    cv::destroyAllWindows();
    return 0;
}