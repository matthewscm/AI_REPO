#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <atomic>
#include <chrono>   // <-- ADD THIS

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        class_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "bottle_class_id", 10,
            std::bind(&CameraNode::class_id_callback, this, std::placeholders::_1));


        bbox_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bottle_bboxes", 10,
            std::bind(&CameraNode::bbox_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/camera/color/image_raw", 10,
            std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

        // Initialize timestamp
        last_bbox_time_ = this->now();
    }

    ~CameraNode()
    {
        running_ = false;
        cv::destroyAllWindows();
    }

private:

    // ───────────────────── CALLBACKS ─────────────────────

    void class_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        bottle_class_id_ = msg->data;
    }

    void bbox_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        bboxes_.clear();

        // Update last received time
        last_bbox_time_ = this->now();

        // format: [x, y, w, h, conf, x, y, w, h, conf, ...]
        for (size_t i = 0; i + 4 < msg->data.size(); i += 5)
        {
            cv::Rect box(
                (int)msg->data[i],
                (int)msg->data[i + 1],
                (int)msg->data[i + 2],
                (int)msg->data[i + 3]
            );

            bboxes_.push_back(box);
        }
    }

    // ───────────────────── TEXT ─────────────────────

    void display_text(cv::Mat &frame,
                      const std::string &text,
                      const cv::Point &position = cv::Point(-1, -1),
                      double font_scale = 1.0,
                      const cv::Scalar &colour = cv::Scalar(0, 255, 0))
    {
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        int thickness = 2;

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        baseline += thickness;

        cv::Point text_org = position;
        if (position.x == -1 && position.y == -1)
        {
            text_org = cv::Point(frame.cols - text_size.width - 10, frame.rows - 10);
        }

        cv::putText(frame, text, text_org, font_face, font_scale, colour, thickness);
    }

    // ───────────────────── IMAGE CALLBACK ─────────────────────

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!running_) return;

        cv::Mat frame;

        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // ───────────────── TIMEOUT CHECK ─────────────────

        double seconds_since_bbox =
            (this->now() - last_bbox_time_).seconds();

        if (seconds_since_bbox > 10.0)
        {
            if (!bboxes_.empty())
            {
                RCLCPP_WARN(this->get_logger(),
                            "No bounding boxes received for 5 seconds, navigating");

                bboxes_.clear();
            }

            bottle_class_id_ = -1;
        }

        display_text(frame, "Live Feed");

        // ── CLASS LABEL ──
        if (bottle_class_id_ == 0)
        {
            label_text = "Red";
            circle_color = cv::Scalar(0, 0, 255);
        }
        else if (bottle_class_id_ == 1)
        {
            label_text = "Green";
            circle_color = cv::Scalar(0, 255, 0);
        }
        else if (bottle_class_id_ == 2)
        {
            label_text = "Blue";
            circle_color = cv::Scalar(255, 0, 0);
        }
        else
        {
            label_text = "Navigating";
            circle_color = cv::Scalar(0, 255, 255);
        }

        display_text(frame, label_text, cv::Point(10, 30), 1.5, circle_color);

        // ── BOUNDING BOXES ──
        for (const auto &box : bboxes_)
        {
            cv::rectangle(frame, box, circle_color, 2);
        }

        auto out_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        pub_->publish(*out_msg);
    }

    // ───────────────────── SHUTDOWN ─────────────────────

    void shutdown_node()
    {
        running_ = false;
        rclcpp::shutdown();
    }

private:

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;

    int bottle_class_id_ = -1;
    geometry_msgs::msg::Point center_;

    std::string label_text = "No Bottle Detected/Unknown Bottle Type";
    cv::Scalar circle_color = cv::Scalar(0, 255, 255);

    std::vector<cv::Rect> bboxes_;

    // ADD THIS
    rclcpp::Time last_bbox_time_;

    std::atomic<bool> running_{true};
};


// ───────────────────── MAIN ─────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <geometry_msgs/msg/point.hpp>

// #include <vector>
// #include <atomic>

// class CameraNode : public rclcpp::Node
// {
// public:
//     CameraNode() : Node("camera_node")
//     {
//         pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

//         class_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//             "bottle_class_id", 10,
//             std::bind(&CameraNode::class_id_callback, this, std::placeholders::_1));

//         center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
//             "bottle_center", 10,
//             std::bind(&CameraNode::center_callback, this, std::placeholders::_1));

//         bbox_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//             "bottle_bboxes", 10,
//             std::bind(&CameraNode::bbox_callback, this, std::placeholders::_1));

//         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "camera/camera/color/image_raw", 10,
//             std::bind(&CameraNode::image_callback, this, std::placeholders::_1));
//     }

//     ~CameraNode()
//     {
//         running_ = false;
//         cv::destroyAllWindows();
//     }

// private:

//     // ───────────────────── CALLBACKS ─────────────────────

//     void class_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
//     {
//         bottle_class_id_ = msg->data;
//     }

//     void center_callback(const geometry_msgs::msg::Point::SharedPtr msg)
//     {
//         center_ = *msg;
//     }

//     void bbox_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
//     {
//         bboxes_.clear();

//         // format: [x, y, w, h, conf, x, y, w, h, conf, ...]
//         for (size_t i = 0; i + 4 < msg->data.size(); i += 5)
//         {
//             cv::Rect box(
//                 (int)msg->data[i],
//                 (int)msg->data[i + 1],
//                 (int)msg->data[i + 2],
//                 (int)msg->data[i + 3]
//             );

//             bboxes_.push_back(box);
//         }
//     }

//     // ───────────────────── TEXT ─────────────────────

//     void display_text(cv::Mat &frame,
//                       const std::string &text,
//                       const cv::Point &position = cv::Point(-1, -1),
//                       double font_scale = 1.0,
//                       const cv::Scalar &colour = cv::Scalar(0, 255, 0))
//     {
//         int font_face = cv::FONT_HERSHEY_SIMPLEX;
//         int thickness = 2;

//         int baseline = 0;
//         cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
//         baseline += thickness;

//         cv::Point text_org = position;
//         if (position.x == -1 && position.y == -1)
//         {
//             text_org = cv::Point(frame.cols - text_size.width - 10, frame.rows - 10);
//         }

//         cv::putText(frame, text, text_org, font_face, font_scale, colour, thickness);
//     }

//     // ───────────────────── IMAGE CALLBACK ─────────────────────

//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         if (!running_) return;

//         cv::Mat frame;

//         try {
//             frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
//         }
//         catch (cv_bridge::Exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
//             return;
//         }

//         display_text(frame, "Live Feed");

//         // ── CLASS LABEL ──
//         if (bottle_class_id_ == 0)
//         {
//             label_text = "Red";
//             circle_color = cv::Scalar(0, 0, 255);
//         }
//         else if (bottle_class_id_ == 1)
//         {
//             label_text = "Green";
//             circle_color = cv::Scalar(0, 255, 0);
//         }
//         else if (bottle_class_id_ == 2)
//         {
//             label_text = "Blue";
//             circle_color = cv::Scalar(255, 0, 0);
//         }
//         else
//         {
//             label_text = "Processing";
//             circle_color = cv::Scalar(0, 255, 255);
//         }

//         display_text(frame, label_text, cv::Point(10, 30), 1.5, circle_color);

//         // ── CENTER POINT ──
//         if (bottle_class_id_ != -1)
//         {
//             cv::circle(frame,
//                        cv::Point((int)center_.x, (int)center_.y),
//                        10,
//                        circle_color,
//                        2);
//         }

//         // ── BOUNDING BOXES ──
//         for (const auto &box : bboxes_)
//         {
//             cv::rectangle(frame, box, circle_color, 2);
//         }

//         auto out_msg =
//             cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

//         pub_->publish(*out_msg);
//     }

//     // ───────────────────── SHUTDOWN ─────────────────────

//     void shutdown_node()
//     {
//         running_ = false;
//         rclcpp::shutdown();
//     }

// private:

//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_id_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
//     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;

//     int bottle_class_id_ = -1;
//     geometry_msgs::msg::Point center_;

//     std::string label_text = "No Bottle Detected/Unknown Bottle Type";
//     cv::Scalar circle_color = cv::Scalar(0, 255, 255);

//     std::vector<cv::Rect> bboxes_;

//     std::atomic<bool> running_{true};
// };


// // ───────────────────── MAIN ─────────────────────

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     cv::destroyAllWindows();
//     return 0;
// }
