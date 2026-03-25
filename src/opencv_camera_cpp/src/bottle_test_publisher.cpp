#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>

using namespace std::chrono_literals;

class BottleTestPublisher : public rclcpp::Node
{
public:
    BottleTestPublisher() : Node("bottle_test_publisher")
    {
        class_pub_ = this->create_publisher<std_msgs::msg::Int32>("bottle_class_id", 10);
        center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("bottle_center", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&BottleTestPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        
        // --- Publish class (colour) ---
        std_msgs::msg::Int32 class_msg;

        // Cycle through colours: 0=Red, 1=Green, 2=Blue
        class_msg.data = counter_ % 3;
        class_pub_->publish(class_msg);

        // --- Publish position ---
        geometry_msgs::msg::Point point_msg;

        // Move in a simple pattern
        point_msg.x = 100 + 50 * std::sin(counter_ * 0.5);
        point_msg.y = 100 + 50 * std::cos(counter_ * 0.5);
        point_msg.z = 0.0;

        center_pub_->publish(point_msg);

        // Log
        RCLCPP_INFO(this->get_logger(),
            "Published -> class: %d | x: %.2f, y: %.2f",
            class_msg.data, point_msg.x, point_msg.y);

        counter_++;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr class_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_ = 0;
};

// ---------------- Main ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleTestPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}