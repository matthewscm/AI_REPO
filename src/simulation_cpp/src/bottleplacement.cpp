#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>
#include <string>
#include <map>
#include <random>

class BottleGridSpawner : public rclcpp::Node {
public:
    BottleGridSpawner() : Node("bottle_grid_spawner") 
    {
        // Bottle STL resource paths
        bottle_meshes_ = {
            {"green", "package://simulation_cpp/meshes/bottle_green.stl"},
            {"red",   "package://simulation_cpp/meshes/bottle_red.stl"},
            {"blue",  "package://simulation_cpp/meshes/bottle_blue.stl"}
        };

        // RGBA Colours
        color_map_ = {
            {"green", makeColor(0.1f, 0.8f, 0.1f)},
            {"red",   makeColor(0.8f, 0.1f, 0.1f)},
            {"blue",  makeColor(0.1f, 0.2f, 0.9f)}
        };

        // Publisher for PlanningScene diffs (Humble-compatible colouring)
        planning_scene_pub_ =
            this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Trigger spawning after a short delay
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BottleGridSpawner::spawn_grid, this));
    }

private:

    // Helper to construct RGBA colour
    std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f)
    {
        std_msgs::msg::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        return c;
    }

    void spawn_grid()
    {
        timer_->cancel(); // Only spawn once

        int rows = 4, cols = 4;
        double spacing = 0.08;
        double origin_x = -0.42, origin_y = -0.12;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<moveit_msgs::msg::ObjectColor> object_colors;

        // Collect bottle types
        std::vector<std::string> types;
        for (auto const& [name, _] : bottle_meshes_)
            types.push_back(name);

        // Random selection engine
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, types.size() - 1);

        // ----- GRID LOOP -----
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c)
            {
                std::string bottle_type = types[dis(gen)];
                std::string mesh_path = bottle_meshes_[bottle_type];

                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = "base_link";
                obj.id = "bottle_" + std::to_string(r) + "_" +
                         std::to_string(c) + "_" + bottle_type;

                // Load mesh
                shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
                if (!m) {
                    RCLCPP_ERROR(this->get_logger(), "Could not load mesh: %s", mesh_path.c_str());
                    continue;
                }

                // Convert mesh → ROS message
                shapes::ShapeMsg shape_msg;
                shape_msgs::msg::Mesh mesh_msg;
                constructMsgFromShape(m, shape_msg);
                mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

                delete m; // cleanup

                // Scale mesh
                double scale = 0.0123;
                for (auto &v : mesh_msg.vertices) {
                    v.x *= scale;
                    v.y *= scale;
                    v.z *= scale;
                }

                // Pose
                geometry_msgs::msg::Pose pose;
                pose.orientation.w = 1.0;
                pose.position.x = origin_x + (r * spacing);
                pose.position.y = origin_y + (c * spacing);
                pose.position.z = 0.0;

                obj.meshes.push_back(mesh_msg);
                obj.mesh_poses.push_back(pose);
                obj.operation = obj.ADD;

                collision_objects.push_back(obj);

                // Register object colour
                moveit_msgs::msg::ObjectColor col;
                col.id = obj.id;
                col.color = color_map_[bottle_type];
                object_colors.push_back(col);
            }
        }

        // Apply collision objects
        planning_scene_interface_.applyCollisionObjects(collision_objects);

        // ----- APPLY COLOURS (HUMBLE) -----
        moveit_msgs::msg::PlanningScene ps_msg;
        ps_msg.is_diff = true;
        ps_msg.object_colors = object_colors;

        planning_scene_pub_->publish(ps_msg);

        RCLCPP_INFO(this->get_logger(), "Spawned coloured + scaled bottle grid.");
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::map<std::string, std::string> bottle_meshes_;
    std::map<std::string, std_msgs::msg::ColorRGBA> color_map_;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleGridSpawner>());
    rclcpp::shutdown();
    return 0;
}