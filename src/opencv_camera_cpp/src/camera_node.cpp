#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Publisher for ROS2 camera topic
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // Open the default camera
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            rclcpp::shutdown();
        }

        // Timer for publishing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&CameraNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
            return;
        }

        // --- Show live feed in OpenCV window ---
        cv::imshow("Live Camera Feed", frame);
        cv::waitKey(1);

        // --- Publish to ROS2 topic ---
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

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