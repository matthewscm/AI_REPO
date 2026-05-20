#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ===========================================================================
// BottleTFSpawner
//
// Subscribes to /bottle_center_average (geometry_msgs/Point in camera_depth_optical_frame)
// Transforms each point to the world frame using TF2
// Spawns a non-collision cylinder in the MoveIt planning scene at the
// transformed position so you can visually verify the transform is correct
//
// Cylinders are named "tf_bottle_<N>" and accumulate in the scene.
// Send an empty point (0,0,0) to clear all spawned cylinders.
// ===========================================================================

class BottleTFSpawner : public rclcpp::Node
{
public:
    BottleTFSpawner()
    : Node("bottle_tf_spawner"), spawn_count_(0)
    {
        // TF buffer and listener — needs the node's clock
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for MoveIt planning scene diffs
        scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "planning_scene", 10);

        // Subscriber — point arrives in camera_depth_optical_frame
        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "bottle_center_average", 10,
            std::bind(&BottleTFSpawner::pointCallback, this, std::placeholders::_1));

        // Declare parameters so they can be overridden at launch
        this->declare_parameter("source_frame",  "camera_depth_optical_frame");
        this->declare_parameter("target_frame",  "world");
        this->declare_parameter("cylinder_radius", 0.035);  // metres
        this->declare_parameter("cylinder_height", 0.150);  // metres

        RCLCPP_INFO(this->get_logger(),
            "BottleTFSpawner ready. Listening on /bottle_center_average");
        RCLCPP_INFO(this->get_logger(),
            "Transforming %s -> %s",
            this->get_parameter("source_frame").as_string().c_str(),
            this->get_parameter("target_frame").as_string().c_str());
        RCLCPP_INFO(this->get_logger(),
            "Send (0,0,0) to clear all spawned cylinders.");
    }

private:
    // ── Members ──────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr    point_sub_;
    int spawn_count_;

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Removes all cylinders spawned by this node from the planning scene
    void clearAll()
    {
        if (spawn_count_ == 0) return;
        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        for (int i = 0; i < spawn_count_; i++) {
            moveit_msgs::msg::CollisionObject rm;
            rm.id        = "tf_bottle_" + std::to_string(i);
            rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            rm.header.frame_id = this->get_parameter("target_frame").as_string();
            diff.world.collision_objects.push_back(rm);
        }
        scene_pub_->publish(diff);
        RCLCPP_INFO(this->get_logger(), "Cleared %d cylinder(s).", spawn_count_);
        spawn_count_ = 0;
    }

    // Spawns a cylinder at (x, y, z) in the target frame.
    // The object is added as a collision object but immediately set to
    // OBJECT_IS_KNOWN (non-attached) — MoveIt will show it in RViz but
    // treat it as a known object, not an obstacle, until you promote it.
    void spawnCylinder(double x, double y, double z)
    {
        const std::string frame  = this->get_parameter("target_frame").as_string();
        const double radius = this->get_parameter("cylinder_radius").as_double();
        const double height = this->get_parameter("cylinder_height").as_double();
        const std::string id = "tf_bottle_" + std::to_string(spawn_count_);

        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = frame;
        obj.header.stamp    = this->now();
        obj.id              = id;

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.CYLINDER;
        prim.dimensions.resize(2);
        prim.dimensions[prim.CYLINDER_HEIGHT] = height;
        prim.dimensions[prim.CYLINDER_RADIUS] = radius;

        geometry_msgs::msg::Pose pose;
        pose.position.x    = x;
        pose.position.y    = y;
        pose.position.z    = z + height / 2.0;  // lift so base sits at detected z
        pose.orientation.w = 1.0;

        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        obj.operation = obj.ADD;

        // Publish as a planning scene diff so it appears in RViz immediately
        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.world.collision_objects.push_back(obj);
        scene_pub_->publish(diff);

        RCLCPP_INFO(this->get_logger(),
            "Spawned '%s' at world (%.3f, %.3f, %.3f)",
            id.c_str(), x, y, z);
        spawn_count_++;
    }

    // ── Callback ──────────────────────────────────────────────────────────────
    void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // (0,0,0) is the clear signal
        if (msg->x == 0.0 && msg->y == 0.0 && msg->z == 0.0) {
            clearAll();
            return;
        }

        const std::string source_frame = this->get_parameter("source_frame").as_string();
        const std::string target_frame = this->get_parameter("target_frame").as_string();

        // Build a stamped point in the source frame
        geometry_msgs::msg::PointStamped point_in;
        point_in.header.frame_id = source_frame;
        point_in.header.stamp    = this->now();
        point_in.point           = *msg;

        // Transform to world frame
        geometry_msgs::msg::PointStamped point_out;
        try {
            // Wait up to 1 second for the transform to become available
            point_out = tf_buffer_->transform(
                point_in, target_frame,
                tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                "TF transform failed (%s -> %s): %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Point in %s: (%.3f, %.3f, %.3f) -> %s: (%.3f, %.3f, %.3f)",
            source_frame.c_str(),
            msg->x, msg->y, msg->z,
            target_frame.c_str(),
            point_out.point.x, point_out.point.y, point_out.point.z);

        spawnCylinder(point_out.point.x, point_out.point.y, point_out.point.z);
    }
};

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottleTFSpawner>());
    rclcpp::shutdown();
    return 0;
}