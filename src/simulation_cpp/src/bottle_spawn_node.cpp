#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <map>
#include <string>
#include <vector>

class BottleCameraSpawner : public rclcpp::Node {
public:
    BottleCameraSpawner() : Node("bottle_camera_spawner"), bottle_count_(0), last_class_id_(-1)
    {
        // Path to your bottle STL
        mesh_path_ = "package://simulation_cpp/meshes/bottle.stl";

        // --- Updated ID Mapping (0: Red, 1: Green, 2: Blue) ---
        id_to_color_name_ = {
            {0, "red"},
            {1, "green"},
            {2, "blue"}
        };

        // --- Color definitions for MoveIt Visualizer ---
        color_map_ = {
            {"red",    makeColor(0.8f, 0.1f, 0.1f)},
            {"green",  makeColor(0.1f, 0.8f, 0.1f)},
            {"blue",   makeColor(0.1f, 0.2f, 0.9f)}
        };

        // Publisher for updating the PlanningScene colors
        planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Subscriptions to Camera Node topics
        class_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "bottle_class_id", 10, std::bind(&BottleCameraSpawner::class_id_callback, this, std::placeholders::_1));

        center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "bottle_center_world", 10, std::bind(&BottleCameraSpawner::center_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Bottle Spawner Started: Monitoring Red(0), Green(1), and Blue(2)");
    }

private:
    std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f)
    {
        std_msgs::msg::ColorRGBA c;
        c.r = r; c.g = g; c.b = b; c.a = a;
        return c;
    }

    void class_id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_class_id_ = msg->data;
    }

    void center_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Only proceed if we have received a valid class ID first
        if (last_class_id_ != -1 && id_to_color_name_.count(last_class_id_)) {
            spawn_bottle(*msg, id_to_color_name_[last_class_id_]);
        }
    }

    void spawn_bottle(const geometry_msgs::msg::Point& point, const std::string& color_name)
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "base_link"; // Ensure this matches your camera's TF frame
        
        // --- Naming Convention: bottle_no_colour ---
        obj.id = "bottle_" + std::to_string(bottle_count_++) + "_" + color_name;

        // Mesh Loading Logic
        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_);
        if (!m) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", mesh_path_.c_str());
            return;
        }

        shapes::ShapeMsg shape_msg;
        constructMsgFromShape(m, shape_msg);
        shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
        delete m;

        // Apply visual scale (0.0123 per original requirements)
        double scale = 0.0123;
        for (auto& v : mesh_msg.vertices) {
            v.x *= scale; v.y *= scale; v.z *= scale;
        }

        double shifted_height = 0.023; // Adjust this value based on your mesh's original dimensions to ensure it sits on the ground

        geometry_msgs::msg::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = point.z - shifted_height; // Shift Bottle Down to Sit on Ground
        pose.orientation.w = 1.0;

        obj.meshes.push_back(mesh_msg);
        obj.mesh_poses.push_back(pose);
        obj.operation = obj.ADD;

        // 1. Apply geometry to the Planning Scene
        planning_scene_interface_.applyCollisionObject(obj);

        // 2. Apply the color via a PlanningScene diff
        moveit_msgs::msg::PlanningScene ps_msg;
        ps_msg.is_diff = true;
        moveit_msgs::msg::ObjectColor col;
        col.id = obj.id;
        col.color = color_map_[color_name];
        ps_msg.object_colors.push_back(col);
        
        planning_scene_pub_->publish(ps_msg);

        RCLCPP_INFO(this->get_logger(), "Spawned: %s at [%.2f, %.2f, %.2f]", 
                    obj.id.c_str(), point.x, point.y, point.z);
    }

    // Members
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::string mesh_path_;
    std::map<int, std::string> id_to_color_name_;
    std::map<std::string, std_msgs::msg::ColorRGBA> color_map_;
    int bottle_count_;
    int last_class_id_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr class_id_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleCameraSpawner>());
    rclcpp::shutdown();
    return 0;
}
