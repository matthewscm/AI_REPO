#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/variant.hpp>

class MultiMeshSpawner : public rclcpp::Node {
public:
    MultiMeshSpawner() : Node("multi_mesh_spawner") {
        mesh_tasks_ = {
            {"crate_recycle", "package://simulation_cpp/meshes/crate_recycle.STL", 0.3, 0.3, 0.0},
            {"crate_discard", "package://simulation_cpp/meshes/crate_discard.STL", 0.3, -0.3, 0.0}
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&MultiMeshSpawner::spawn_environment, this));
            
        RCLCPP_INFO(this->get_logger(), "Environment Node started. Waiting to spawn meshes and ground...");
    }

private:
    struct MeshTask {
        std::string id;
        std::string path;
        double x, y, z;
    };

    void spawn_environment() {
        timer_->cancel();
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        // --- 1. ADD THE GROUND PLANE (Solid Primitive) ---
        moveit_msgs::msg::CollisionObject ground;
        ground.header.frame_id = "base_link";
        ground.id = "ground_plane";

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {1.0, 1.0, 0.01}; // X, Y, Z sizes in meters

        geometry_msgs::msg::Pose ground_pose;
        ground_pose.orientation.w = 1.0;
        ground_pose.position.x = 0.0; // Centered relative to your crates
        ground_pose.position.y = 0.0;
        ground_pose.position.z = -0.006; // Half of thickness to keep top at z=0

        ground.primitives.push_back(box);
        ground.primitive_poses.push_back(ground_pose);
        ground.operation = ground.ADD;
        collision_objects.push_back(ground);

        // --- 2. ADD THE MESHES ---
        for (const auto& task : mesh_tasks_) {
            moveit_msgs::msg::CollisionObject obj;
            obj.header.frame_id = "base_link"; 
            obj.id = task.id;

            shapes::Mesh* m = shapes::createMeshFromResource(task.path);
            if (!m) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load: %s", task.path.c_str());
                continue;
            }

            shapes::ShapeMsg shape_msg;
            shapes::constructMsgFromShape(m, shape_msg);
            auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
            delete m;

            // Apply Scale (mm to m)
            double scale = 0.001; 
            for (auto& vertex : mesh_msg.vertices) {
                vertex.x *= scale; vertex.y *= scale; vertex.z *= scale;
            }

            geometry_msgs::msg::Pose pose;
            pose.orientation.w = 1.0;
            pose.position.x = task.x;
            pose.position.y = task.y;
            pose.position.z = task.z;

            obj.meshes.push_back(mesh_msg);
            obj.mesh_poses.push_back(pose);
            obj.operation = obj.ADD;

            collision_objects.push_back(obj);
        }

        psi_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(this->get_logger(), "Spawned ground and %zu meshes.", collision_objects.size() - 1);
    }

    moveit::planning_interface::PlanningSceneInterface psi_;
    std::vector<MeshTask> mesh_tasks_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiMeshSpawner>());
    rclcpp::shutdown();
    return 0;
}