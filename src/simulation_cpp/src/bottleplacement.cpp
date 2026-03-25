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
        // --- Single STL file for all bottles ---
        mesh_path_ = "package://simulation_cpp/meshes/bottle.stl";

        // --- Bottle types (keys only) ---
        bottle_types_ = {"green", "red", "blue", "orange", "black"};

        // --- Colours for each bottle type ---
        color_map_ = {
            {"green",  makeColor(0.1f, 0.8f, 0.1f)},
            {"red",    makeColor(0.8f, 0.1f, 0.1f)},
            {"blue",   makeColor(0.1f, 0.2f, 0.9f)},
            {"orange", makeColor(1.0f, 0.5f, 0.0f)},
            {"black",  makeColor(0.05f, 0.05f, 0.05f)}
        };

        // Publisher for PlanningScene colour diffs
        planning_scene_pub_ =
            this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Timer to trigger once
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BottleGridSpawner::spawn_grid, this));
    }

private:

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
        timer_->cancel();

        int rows = 4, cols = 4;
        double spacing = 0.08;
        double origin_x = -0.42, origin_y = -0.12;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<moveit_msgs::msg::ObjectColor> object_colors;

        // Random selection engine
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, bottle_types_.size() - 1);

        // --- Build a 4x4 grid of bottles ---
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c)
            {
                // Pick a bottle type (colour)
                std::string bottle_type = bottle_types_[dis(gen)];

                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = "base_link";
                obj.id = "bottle_" + std::to_string(r) + "_" +
                         std::to_string(c) + "_" + bottle_type;

                // Load mesh ONCE for all bottles
                shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_);
                if (!m) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Could not load mesh: %s",
                                 mesh_path_.c_str());
                    continue;
                }

                // Convert mesh → ROS message
                shapes::ShapeMsg shape_msg;
                shape_msgs::msg::Mesh mesh_msg;

                constructMsgFromShape(m, shape_msg);
                mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
                delete m;

                // Scale mesh
                double scale = 0.0123;
                for (auto& v : mesh_msg.vertices) {
                    v.x *= scale;
                    v.y *= scale;
                    v.z *= scale;
                }

                // Pose
                geometry_msgs::msg::Pose pose;
                pose.orientation.w = 1.0;
                pose.position.x = origin_x + r * spacing;
                pose.position.y = origin_y + c * spacing;
                pose.position.z = 0.0;

                obj.meshes.push_back(mesh_msg);
                obj.mesh_poses.push_back(pose);
                obj.operation = obj.ADD;

                collision_objects.push_back(obj);

                // --- Assign colour ---
                moveit_msgs::msg::ObjectColor col;
                col.id = obj.id;
                col.color = color_map_[bottle_type];
                object_colors.push_back(col);
            }
        }

        // Apply meshes
        planning_scene_interface_.applyCollisionObjects(collision_objects);

        // Apply colours via PlanningScene diff
        moveit_msgs::msg::PlanningScene ps_msg;
        ps_msg.is_diff = true;
        ps_msg.object_colors = object_colors;

        planning_scene_pub_->publish(ps_msg);

        RCLCPP_INFO(this->get_logger(),
            "Spawned 4x4 grid of coloured bottles using one STL file.");
    }

    // ---- Members ----
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::string mesh_path_;
    std::vector<std::string> bottle_types_;
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