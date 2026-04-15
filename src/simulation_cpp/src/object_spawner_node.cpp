#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/msg/shape_msgs.h>

class ObjectSpawnerNode : public rclcpp::Node
{
public:
    ObjectSpawnerNode() : Node("object_spawner")
    {
        RCLCPP_INFO(get_logger(), "ObjectSpawner node started. Waiting for MoveIt...");

        // Allow RViz + MoveIt to finish loading
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&ObjectSpawnerNode::spawn_objects, this)
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    void spawn_objects()
    {
        static bool spawned = false;
        if (spawned)
            return;

        spawned = true;

        moveit::planning_interface::PlanningSceneInterface psi;

        std::vector<moveit_msgs::msg::CollisionObject> objects;

        // ----------------------------------------------------
        // 1. Spawn a BOX
        // ----------------------------------------------------
        moveit_msgs::msg::CollisionObject box;
        box.id = "test_box";
        box.header.frame_id = "world";

        box.primitives.resize(1);
        box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        box.primitives[0].dimensions = {0.2, 0.2, 0.2};  // x,y,z

        box.primitive_poses.resize(1);
        box.primitive_poses[0].position.x = 0.4;
        box.primitive_poses[0].position.y = 0.0;
        box.primitive_poses[0].position.z = 0.1;

        box.operation = box.ADD;
        objects.push_back(box);

        // ----------------------------------------------------
        // 2. Spawn a CYLINDER
        // ----------------------------------------------------
        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.id = "test_cylinder";
        cylinder.header.frame_id = "world";

        cylinder.primitives.resize(1);
        cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder.primitives[0].dimensions = {0.4, 0.05};  // height, radius

        cylinder.primitive_poses.resize(1);
        cylinder.primitive_poses[0].position.x = 0.2;
        cylinder.primitive_poses[0].position.y = -0.3;
        cylinder.primitive_poses[0].position.z = 0.2;

        cylinder.operation = cylinder.ADD;
        objects.push_back(cylinder);

        // ----------------------------------------------------
        // Add all objects
        // ----------------------------------------------------
        psi.applyCollisionObjects(objects);

        RCLCPP_INFO(this->get_logger(), "Spawned %lu objects in RViz", objects.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}