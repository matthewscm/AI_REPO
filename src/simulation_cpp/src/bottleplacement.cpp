#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>


#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>
#include <string>
#include <map>
#include <random>

class BottleGridSpawner : public rclcpp::Node {
public:
    BottleGridSpawner() : Node("bottle_grid_spawner") {
        // Map now stores the file path to the mesh instead of just height
        // Replace these with your actual mesh paths (can use "package://package_name/...")
        bottle_meshes_ = {
            {"green", "package://simulation_cpp/meshes/bottle_green.stl"},
            {"red", "package://simulation_cpp/meshes/bottle_red.stl"},
            {"blue", "package://simulation_cpp/meshes/bottle_blue.stl"}
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&BottleGridSpawner::spawn_grid, this));
    }

private:
    void spawn_grid() {
        timer_->cancel();
        
        int rows = 4, cols = 4;
        double spacing = 0.08;  //0.12
        double origin_x = -0.42, origin_y = -0.12;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<std::string> keys;
        for (auto const& [name, path] : bottle_meshes_) keys.push_back(name);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, keys.size() - 1);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                std::string bottle_type = keys[dis(gen)];
                std::string mesh_path = bottle_meshes_[bottle_type];

                moveit_msgs::msg::CollisionObject obj;
                obj.header.frame_id = "base_link";
                obj.id = "bottle_" + std::to_string(r) + "_" + std::to_string(c) + "_" + bottle_type;

                // --- MESH LOADING LOGIC ---
                // 1. Load mesh
                shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

                if (!m) {
                    RCLCPP_ERROR(this->get_logger(), "Could not load mesh: %s", mesh_path.c_str());
                    return;
                }

                // 2. Convert shape → message
                shape_msgs::msg::Mesh mesh_msg;
                shapes::ShapeMsg mesh_shape_msg;

                // ✔ Correct ROS2 namespace:
                constructMsgFromShape(m, mesh_shape_msg);   // ✔ global namespace

                // 3. Extract Mesh
                mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_shape_msg);

                
                // --- SCALE THE MESH ---
                double scale = 0.0123;   // ← change this value to resize bottles

                for (auto &vertex : mesh_msg.vertices) {
                    vertex.x *= scale;
                    vertex.y *= scale;
                    vertex.z *= scale;
                }


                delete m;

                geometry_msgs::msg::Pose pose;
                pose.orientation.w = 1.0;
                pose.position.x = origin_x + (r * spacing);
                pose.position.y = origin_y + (c * spacing);
                // Note: Meshes usually have their origin at the bottom, 
                // so Z might be 0.0 instead of height/2.
                pose.position.z = 0.0; 

                obj.meshes.push_back(mesh_msg);
                obj.mesh_poses.push_back(pose);
                obj.operation = obj.ADD;

                collision_objects.push_back(obj);
            }
        }

        planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Spawned mesh grid.");
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::map<std::string, std::string> bottle_meshes_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleGridSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}