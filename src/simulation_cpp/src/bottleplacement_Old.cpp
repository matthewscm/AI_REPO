#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <string>
#include <map>
#include <random>

class BottleGridSpawner : public rclcpp::Node {
public:
    BottleGridSpawner() : Node("bottle_grid_spawner") {
        bottle_types_ = {
            {"green", 0.20},
            {"red", 0.25},
            {"blue", 0.15},
            {"yellow", 0.30},
            {"purple", 0.18}
        };

        // We use a timer because PlanningSceneInterface needs a second 
        // to connect to the move_group node after the node starts.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&BottleGridSpawner::spawn_grid, this));
    }

private:
    void spawn_grid() {
        timer_->cancel(); // Only run this once
        
        int rows = 4;
        int cols = 4;
        double spacing = 0.08; // 8cm spacing between bottles
        double origin_x = -0.42;
        double origin_y = -0.12;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        std::vector<std::string> keys;
        for (auto const& [name, height] : bottle_types_) keys.push_back(name);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, keys.size() - 1);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = "base_link"; 

                // 1. Get the random bottle type name (e.g., "red")
                std::string bottle_type = keys[dis(gen)];
                double height = bottle_types_[bottle_type];

                // 2. Set the ID: bottle_ROW_COL_TYPE
                // Example: bottle_0_1_green
                obj.id = "bottle_" + std::to_string(r) + "_" + std::to_string(c) + "_" + bottle_type; // ID for each bottle with Type name

                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions = {0.07, 0.07, height}; // 7cm width x 7cm depth, height based on type

                geometry_msgs::msg::Pose pose;
                pose.orientation.w = 1.0;
                pose.position.x = origin_x + (r * spacing);
                pose.position.y = origin_y + (c * spacing);
                // Offset Z by half height so the bottle sits ON the ground, 
                // since MoveIt places the center of the box at the coordinate.
                pose.position.z = height / 2.0; 

                obj.primitives.push_back(primitive);
                obj.primitive_poses.push_back(pose);
                obj.operation = obj.ADD;

                collision_objects.push_back(obj);
            }
        }

        // // --- 2. Add Recycle and Discard Boxes ---
        // auto create_crate = [&](std::string id, double x, double y) {
        //     moveit_msgs::msg::CollisionObject crate;
        //     crate.header.frame_id = "base_link";
        //     crate.id = id;

        //     shape_msgs::msg::SolidPrimitive crate_shape;
        //     crate_shape.type = crate_shape.BOX;
        //     // Dimensions: 20cm x 20cm x 10cm height
        //     crate_shape.dimensions = {0.2, 0.2, 0.1};

        //     geometry_msgs::msg::Pose crate_pose;
        //     crate_pose.orientation.w = 1.0;
        //     crate_pose.position.x = x;
        //     crate_pose.position.y = y;
        //     crate_pose.position.z = 0.05; // Half of height (0.1/2)

        //     crate.primitives.push_back(crate_shape);
        //     crate.primitive_poses.push_back(crate_pose);
        //     crate.operation = crate.ADD;
        //     return crate;
        //};


        planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Successfully sent %zu bottles to Planning Scene.", collision_objects.size());

        // collision_objects.push_back(create_crate("recycle", -0.3, -0.3));
        // collision_objects.push_back(create_crate("discard", -0.3, 0.3));

        // planning_scene_interface_.applyCollisionObjects(collision_objects);
        // RCLCPP_INFO(this->get_logger(), "Spawned grid and crates to Planning Scene.");

    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::map<std::string, double> bottle_types_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleGridSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}