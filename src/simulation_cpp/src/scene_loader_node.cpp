#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <jsoncpp/json/json.h>
#include <fstream>

class SceneLoader : public rclcpp::Node
{
public:
    SceneLoader(const std::string &scene_path)
        : Node("scene_loader"), scene_path_(scene_path)
    {
        pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Correct timer syntax
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            this { this->loadScene(); }
        );

    }

private:
    void loadScene()
    {
        RCLCPP_INFO(this->get_logger(), "Loading scene: %s", scene_path_.c_str());

        std::ifstream f(scene_path_);
        if (!f.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open scene file.");
            return;
        }

        Json::Value root;
        f >> root;

        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;

        pub_->publish(ps);

        RCLCPP_INFO(this->get_logger(), "Scene loaded.");
    }

    std::string scene_path_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("scene_loader"), "Usage: scene_loader_node <scene_file>");
        return 1;
    }

    auto node = std::make_shared<SceneLoader>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}