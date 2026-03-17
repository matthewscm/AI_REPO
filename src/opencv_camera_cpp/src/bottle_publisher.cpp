#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

using namespace std::chrono_literals;

class BottlePublisher : public rclcpp::Node
{
public:
    BottlePublisher() : Node("bottle_publisher")
    {
        // Publisher for the flag
        pub_ = this->create_publisher<std_msgs::msg::Bool>("bottle_detected", 10);

        // Timer to toggle the flag every 2 seconds
        timer_ = this->create_wall_timer(
            2s, std::bind(&BottlePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::Bool();
        // Toggle the flag on/off
        bottle_detected_ = !bottle_detected_;
        msg.data = bottle_detected_;
        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published bottle_detected: %s", 
                    bottle_detected_ ? "true" : "false");
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool bottle_detected_ = false;
};

// ---------------- Main ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottlePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}