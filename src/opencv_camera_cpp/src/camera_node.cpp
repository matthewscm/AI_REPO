#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/bool.hpp> 
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point.hpp>

class CameraNode : public rclcpp::Node
{
public:
// Constructor

    CameraNode() : Node("camera_node")
    {
        // Publisher for ROS2 camera topic
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        class_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "bottle_class_id", 10,
            std::bind(&CameraNode::class_id_callback, this, std::placeholders::_1)
        );

        center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "bottle_center", 10,
            std::bind(&CameraNode::center_callback, this, std::placeholders::_1)
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
    // Callback for class ID updates
    void class_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {        
        bottle_class_id_ = msg->data;
    }
    // Callback for center point updates
    void center_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        center_ = *msg;
    }
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

        // Choose color and label
        if (bottle_class_id_ == 0) {
            label_text = "Red";
            circle_color = cv::Scalar(0, 0, 255);  // BGR
        } 
        else if (bottle_class_id_ == 1) {
            label_text = "Green";
            circle_color = cv::Scalar(0, 255, 0);
        } 
        else if (bottle_class_id_ == 2) {
            label_text = "Blue";
            circle_color = cv::Scalar(255, 0, 0);
        } 
        else {
            label_text = "Processing";
            circle_color = cv::Scalar(0, 255, 255);  // Cyan for unknown
        }

        // Display text
        display_text(frame, label_text, cv::Point(10, 30), 1.5, circle_color);

        // Draw center point if valid
        if (bottle_class_id_ != -1) {
            int radius = 10;       // Slightly bigger circle
            int thickness = 2;     // Slightly visible (positive thickness)
            cv::circle(frame, cv::Point(center_.x, center_.y), radius, circle_color, thickness);
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

        //RCLCPP_INFO(this->get_logger(), "Receiving image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bottle_sub_; //not needed
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_id_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
    // For laptop Camera 
    //cv::VideoCapture cap_;
    //rclcpp::TimerBase::SharedPtr timer_;
    //bool bottle_detected_ = false;
    int bottle_class_id_ = -1;
    geometry_msgs::msg::Point center_;
    std::string label_text = "No Bottle Detected/Unknown Bottle Type";
    cv::Scalar circle_color = cv::Scalar(0, 255, 255);
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