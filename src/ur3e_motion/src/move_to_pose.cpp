#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>
#include <cmath>
#include <optional>
#include <string>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <condition_variable>

// ===========================================================================
// TUNING CONSTANTS
// ===========================================================================
constexpr double OPEN_WIDTH          = 0.085;
constexpr double CLOSED_WIDTH        = 0.035;
constexpr double VELOCITY            = 0.1;
constexpr double ACCELERATION        = 0.1;
constexpr double PLAN_TIME           = 15.0;
constexpr double PLAN_TIME_LONG      = 20.0;
constexpr double BODY_HEIGHT         = 0.180;
constexpr double PRE_PICK_HOVER      = 0.040;
constexpr double TRANSIT_HEIGHT      = 0.55;
constexpr double CARTESIAN_STEP      = 0.003;
constexpr double CARTESIAN_FRACTION  = 0.85;
constexpr double CARTESIAN_FRAC_SOFT = 0.80;
constexpr double IK_TIMEOUT          = 0.50;
constexpr double ELBOW_FLIP_THRESHOLD    = 0.25;
constexpr double DEFAULT_PATH_RATIO      = 2.0;
constexpr double CROSS_FAMILY_PATH_RATIO = 2.5;
constexpr double MAX_SINGLE_JOINT_JUMP   = 2.0;
constexpr int    CARRY_SETTLE_MS         = 400;

// Commands received on the "system_command" ROS topic
constexpr int CMD_NONE    = -1;
constexpr int CMD_HOME    =  0;
constexpr int CMD_START   =  1;
constexpr int CMD_STOP    =  2;
constexpr int CMD_RESUME  =  3;
constexpr int CMD_LOCATE  =  4;
constexpr int CMD_DETECT  =  5;
constexpr int CMD_VIEW2   =  6;
constexpr int CMD_MISSION =  7;

// ===========================================================================
// CRATE DEFINITIONS
// ===========================================================================
struct CrateConfig {
    double cx, cy, pz;
    std::string label;
};

const CrateConfig CRATE_1 = { -0.3,  0.3, 0.34, "crate_1" };
const CrateConfig CRATE_2 = { -0.4, -0.45, 0.34, "crate_2" };

// ===========================================================================
// FIXED ORIENTATIONS & JOINT POSITIONS
// ===========================================================================
struct Quaternion { double x, y, z, w; };
constexpr Quaternion ORI_PICK    = {  0.7027, -0.0792,  0.7026,  0.0792 };
constexpr Quaternion ORI_PLACE   = { -0.1250, -0.6957, -0.1244,  0.6969 }; // crate 1
constexpr Quaternion ORI_PLACE_2 = {  0.262,  -0.680,   0.195,   0.657  }; // crate 2

const std::vector<double> HOME_JOINTS   = {  0.0,    -1.5700,  0.0,    -1.5699, -1.4959,  0.0    };
const std::vector<double> LOCATE_JOINTS = { -0.5104, -1.5700, -0.0087, -0.6864, -1.4961, -0.0001 };
const std::vector<double> VIEW2_JOINTS  = { -0.3776, -2.7936,  0.0097, -0.1026, -1.5821,  0.0513 };
const std::vector<double> CARRY_JOINTS  = { -0.3009, -1.5098,  0.7255, -2.3576, -1.4937,  0.0    };
const std::vector<double> SEED_PLACE_A  = { -3.5748, -1.5544,  0.8426, -2.4283, -1.4928,  0.0    }; // crate 1
const std::vector<double> SEED_PLACE_B  = { -2.4401, -1.6279,  2.0919, -3.5712, -1.3615,  0.0028 }; // crate 2

static const double JOINT_LO[] = { -6.2, -6.2, -3.14, -6.2, -6.2, -6.2 };
static const double JOINT_HI[] = {  6.2,  6.2,  3.14,  6.2,  6.2,  6.2 };

// ===========================================================================
// DATA TYPES
// ===========================================================================
struct IKSolution {
    std::vector<double> joints;
    geometry_msgs::msg::Pose pose;
    std::string name;
};

struct BottlePosition { double x, y, z; std::string id; std::string colour; };
struct PlacePosition  { double x, y, z; std::string id; std::string crate; };

struct ComputedConfig {
    IKSolution pre_pick, pick, carry, pre_place, place_down, post_place;
    double lift_target_z;
    double place_z;
};

enum class MotionStep {
    PRE_PICK, PICK, LIFT, CARRY, PRE_PLACE, LOWER, RELEASE, POST_PLACE, RETURN_CARRY
};

static std::string stepName(MotionStep s) {
    static const char* names[] = {
        "PRE_PICK","PICK","LIFT","CARRY","PRE_PLACE","LOWER","RELEASE","POST_PLACE","RETURN_CARRY"
    };
    return names[static_cast<int>(s)];
}

// ===========================================================================
// COLOUR HELPERS
// ===========================================================================
static std::string extractColour(const std::string & id)
{
    auto pos = id.rfind('_');
    if (pos == std::string::npos || pos + 1 >= id.size()) return "";
    return id.substr(pos + 1);
}

static std::pair<std::vector<BottlePosition>, std::vector<BottlePosition>>
sortBottlesByColour(const std::vector<BottlePosition> & bottles, const rclcpp::Logger & logger)
{
    std::vector<BottlePosition> crate1, crate2;
    std::string primary_colour;
    for (const auto & b : bottles) {
        if (primary_colour.empty()) {
            primary_colour = b.colour;
            RCLCPP_INFO(logger, "[sort] Primary colour: '%s' -> %s", primary_colour.c_str(), CRATE_1.label.c_str());
        }
        if (b.colour == primary_colour) {
            crate1.push_back(b);
            RCLCPP_INFO(logger, "[sort] '%s' (%s) -> %s", b.id.c_str(), b.colour.c_str(), CRATE_1.label.c_str());
        } else {
            crate2.push_back(b);
            RCLCPP_INFO(logger, "[sort] '%s' (%s) -> %s", b.id.c_str(), b.colour.c_str(), CRATE_2.label.c_str());
        }
    }
    return {crate1, crate2};
}

// ===========================================================================
// MATH HELPERS
// ===========================================================================
static double normaliseAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static double angDist(double a, double b) { return std::abs(normaliseAngle(a - b)); }

static geometry_msgs::msg::Pose makePose(double x, double y, double z, const Quaternion & q) {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = z;
    p.orientation.x = q.x; p.orientation.y = q.y; p.orientation.z = q.z; p.orientation.w = q.w;
    return p;
}

// ===========================================================================
// ROBOT STATE
// ===========================================================================
static moveit::core::RobotStatePtr getStateWithRetry(
    moveit::planning_interface::MoveGroupInterface & arm,
    const rclcpp::Logger & logger, int attempts = 10)
{
    for (int i = 0; i < attempts; i++) {
        auto s = arm.getCurrentState(5.0);
        if (s) return s;
        RCLCPP_WARN(logger, "getCurrentState attempt %d/%d failed", i + 1, attempts);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    return nullptr;
}

// ===========================================================================
// INVERSE KINEMATICS
// ===========================================================================
static bool normaliseSolution(IKSolution & sol, const std::vector<double> & seed,
    const rclcpp::Logger & logger)
{
    bool valid = true;
    for (size_t j = 0; j < sol.joints.size() && j < seed.size(); j++) {
        double raw = sol.joints[j], v = raw;
        while (v - seed[j] >  M_PI) v -= 2.0 * M_PI;
        while (v - seed[j] < -M_PI) v += 2.0 * M_PI;
        if (std::abs(v - raw) > 1e-6)
            RCLCPP_INFO(logger, "IK norm [%s] j%zu: %.3f->%.3f", sol.name.c_str(), j, raw, v);
        if (j < 6 && (v < JOINT_LO[j] || v > JOINT_HI[j])) {
            RCLCPP_WARN(logger, "IK REJECT [%s] j%zu=%.3f out of [%.2f,%.2f]",
                sol.name.c_str(), j, v, JOINT_LO[j], JOINT_HI[j]);
            valid = false;
        }
        sol.joints[j] = v;
    }
    return valid;
}

static std::optional<std::pair<IKSolution, IKSolution>>
solveIK(double tx, double ty, double tz, const std::string & side,
        const Quaternion & ori,
        moveit::planning_interface::MoveGroupInterface & arm, const rclcpp::Logger & logger)
{
    auto state = getStateWithRetry(arm, logger);
    if (!state) { RCLCPP_ERROR(logger, "solveIK: no robot state"); return std::nullopt; }

    auto target = makePose(tx, ty, tz, ori);
    auto * jmg  = state->getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");

    std::vector<std::vector<double>> seeds;
    if (side == "place_crate2") {
        seeds = {
            { -2.4401, -1.6279,  2.0919, -3.5712, -1.3615,  0.0028 },
            { -2.4401, -1.6279, -2.0919, -3.5712, -1.3615,  0.0028 },
            { -2.5000, -1.7000,  2.2000, -3.6000, -1.3615,  0.0    },
            { -2.3000, -1.6000,  2.0000, -3.5000, -1.3615,  0.0    },
            { -2.5000, -1.8000,  2.3000, -3.7000, -1.3615,  0.0    },
            { -2.4000, -1.5000,  1.9000, -3.4000, -1.3615,  0.0    },
        };
    } else if (side == "place") {
        seeds = {
            { -3.5748, -1.5544,  0.8426, -2.4283, -1.4928,  0.0 },
            { -3.9699, -1.5688, -0.8426, -2.4219, -1.4928,  0.0 },
            { -2.8000, -1.5500,  0.8000, -2.4000, -1.4928,  0.0 },
            { -3.2000, -1.6000,  1.0000, -2.5000, -1.4928,  0.0 },
            { -3.2000, -1.6000, -1.0000, -2.5000, -1.4928,  0.0 },
            { -2.6464, -1.7567,  1.0767, -2.5000, -1.4726,  0.0 },
        };
    } else {
        double pan = std::atan2(ty, tx) - M_PI / 2.0;
        while (pan >  M_PI) pan -= 2.0 * M_PI;
        while (pan < -M_PI) pan += 2.0 * M_PI;
        RCLCPP_INFO(logger, "IK pick pan=%.3f (%.3f,%.3f)", pan, tx, ty);
        seeds = {
            { pan,    -1.3905,  2.3495, -4.1008, -1.4959, 0.0 },
            { pan,    -1.3905, -2.3495, -4.1008, -1.4959, 0.0 },
            { pan,    -1.2000,  2.2000, -4.0000, -1.4959, 0.0 },
            { pan,    -1.5000,  2.5000, -4.2000, -1.4959, 0.0 },
            { pan,    -1.1000,  2.1000, -3.9000, -1.4959, 0.0 },
            { -0.2993, -1.3905,  2.3495, -4.1008, -1.4959, 0.0 },
            { -0.2993, -1.3905, -2.3495, -4.1008, -1.4959, 0.0 },
        };
    }

    IKSolution sol_up, sol_down;
    bool found_up = false, found_down = false;
    for (size_t si = 0; si < seeds.size() && !(found_up && found_down); si++) {
        const auto & seed = seeds[si];
        state->setJointGroupPositions(jmg, seed);
        state->update();
        if (!state->setFromIK(jmg, target, IK_TIMEOUT)) continue;
        IKSolution sol;
        state->copyJointGroupPositions(jmg, sol.joints);
        sol.pose = target;
        sol.name = (si % 2 == 0) ? "elbow_up" : "elbow_down";
        if (!normaliseSolution(sol, seed, logger)) continue;
        if (!found_up)        { sol.name = "elbow_up";   sol_up   = sol; found_up   = true; }
        else if (!found_down) { sol.name = "elbow_down"; sol_down = sol; found_down = true; }
    }
    if (!found_up && !found_down) {
        RCLCPP_WARN(logger, "IK(%s): no solution (%.3f,%.3f,%.3f)", side.c_str(), tx, ty, tz);
        return std::nullopt;
    }
    if (!found_up)   sol_up   = sol_down;
    if (!found_down) sol_down = sol_up;
    return std::make_pair(sol_up, sol_down);
}

static IKSolution chooseSolution(const std::pair<IKSolution, IKSolution> & sols,
    const std::vector<double> & ref, const std::string & last, const rclcpp::Logger & logger)
{
    auto cost = [&](const IKSolution & s) {
        return angDist(s.joints[0], ref[0]) * 2.5 + angDist(s.joints[1], ref[1]) * 1.5 +
               angDist(s.joints[2], ref[2]) * 1.0 + angDist(s.joints[3], ref[3]) * 0.5;
    };
    const auto & up = sols.first, & down = sols.second;
    double cu = cost(up), cd = cost(down);
    const auto & best  = (cu <= cd) ? up   : down;
    const auto & worse = (cu <= cd) ? down : up;
    if (!last.empty() && last != best.name) {
        double imp = cost(worse) - cost(best);
        if (imp < ELBOW_FLIP_THRESHOLD) return (last == up.name) ? up : down;
        RCLCPP_INFO(logger, "IK: %s->%s (imp %.3f)", last.c_str(), best.name.c_str(), imp);
    }
    return best;
}

static std::optional<IKSolution> solveAndChoose(
    double tx, double ty, double tz, const std::string & side,
    const Quaternion & ori,
    const std::vector<double> & ref, const std::string & last,
    moveit::planning_interface::MoveGroupInterface & arm, const rclcpp::Logger & logger)
{
    auto sols = solveIK(tx, ty, tz, side, ori, arm, logger);
    if (!sols) return std::nullopt;
    return chooseSolution(*sols, ref, last, logger);
}

static std::optional<ComputedConfig> computeConfig(
    const BottlePosition & b, const PlacePosition & p,
    moveit::planning_interface::MoveGroupInterface & arm, const rclcpp::Logger & logger)
{
    const bool is_crate2 = (p.crate == CRATE_2.label);
    const Quaternion & place_ori  = is_crate2 ? ORI_PLACE_2 : ORI_PLACE;
    const std::string   place_side = is_crate2 ? "place_crate2" : "place";
    const auto & place_seed        = is_crate2 ? SEED_PLACE_B  : SEED_PLACE_A;

    RCLCPP_INFO(logger, "--- IK: '%s'(%.3f,%.3f,%.3f)->'%s'(%.3f,%.3f,%.3f) [%s] ---",
        b.id.c_str(), b.x, b.y, b.z, p.id.c_str(), p.x, p.y, p.z, p.crate.c_str());

    ComputedConfig cfg;
    std::string last;

    auto sp = [&](double x, double y, double z, const std::vector<double> & ref)
        -> std::optional<IKSolution> {
        return solveAndChoose(x, y, z, "pick", ORI_PICK, ref, last, arm, logger);
    };
    auto sl = [&](double x, double y, double z, const std::vector<double> & ref)
        -> std::optional<IKSolution> {
        return solveAndChoose(x, y, z, place_side, place_ori, ref, last, arm, logger);
    };

    auto r1 = sp(b.x, b.y, b.z + PRE_PICK_HOVER, HOME_JOINTS);
    if (!r1) { RCLCPP_ERROR(logger, "PRE_PICK unsolvable"); return std::nullopt; }
    cfg.pre_pick = *r1; last = cfg.pre_pick.name;

    auto r2 = sp(b.x, b.y, b.z, cfg.pre_pick.joints);
    if (!r2) { RCLCPP_ERROR(logger, "PICK unsolvable"); return std::nullopt; }
    cfg.pick = *r2; last = cfg.pick.name;

    cfg.carry.joints = CARRY_JOINTS; cfg.carry.name = "carry"; last = "carry";

    auto r4 = sl(p.x, p.y, TRANSIT_HEIGHT, place_seed);
    if (!r4) { RCLCPP_ERROR(logger, "PRE_PLACE unsolvable"); return std::nullopt; }
    cfg.pre_place = *r4; last = cfg.pre_place.name;

    auto r5 = sl(p.x, p.y, p.z, cfg.pre_place.joints);
    if (!r5) { RCLCPP_ERROR(logger, "PLACE_DOWN unsolvable"); return std::nullopt; }
    cfg.place_down = *r5;

    cfg.post_place      = cfg.pre_place;
    cfg.post_place.name = "post_place";
    cfg.lift_target_z   = TRANSIT_HEIGHT;
    cfg.place_z         = p.z;
    RCLCPP_INFO(logger, "--- IK OK ---");
    return cfg;
}

// ===========================================================================
// SCENE / GRIPPER HELPERS
// ===========================================================================
using GripperPub = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;
using ScenePub   = rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr;

static void setGripper(GripperPub & pub, double width,
    const rclcpp::Logger & logger, const std::string & label)
{
    std_msgs::msg::Float64MultiArray msg; msg.data = {width}; pub->publish(msg);
    RCLCPP_INFO(logger, "Gripper %s (%.3f)", label.c_str(), width);
    rclcpp::sleep_for(std::chrono::milliseconds(800));
}

static void addBox(moveit::planning_interface::PlanningSceneInterface & psi,
    const std::string & id, const std::string & frame,
    double x, double y, double z, double sx, double sy, double sz)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame; obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX; prim.dimensions = {sx, sy, sz};
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    pose.orientation.w = 1.0;
    obj.primitives.push_back(prim); obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
}

static void removeObject(moveit::planning_interface::PlanningSceneInterface & psi,
    ScenePub & scene_pub, const std::string & id,
    const std::string & frame, const rclcpp::Logger & logger)
{
    moveit_msgs::msg::PlanningScene diff; diff.is_diff = true;
    moveit_msgs::msg::CollisionObject rm;
    rm.id = id; rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    rm.header.frame_id = frame;
    diff.world.collision_objects.push_back(rm);
    scene_pub->publish(diff);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    psi.removeCollisionObjects({id});
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_INFO(logger, "[scene] removed '%s'", id.c_str());
}

static std::vector<BottlePosition> discoverBottles(
    moveit::planning_interface::PlanningSceneInterface & psi, const rclcpp::Logger & logger)
{
    std::vector<BottlePosition> bottles;
    auto known = psi.getKnownObjectNames();
    auto poses = psi.getObjectPoses(known);
    for (const auto & id : known) {
        if (id.rfind("bottle_", 0) != 0 || poses.find(id) == poses.end()) continue;
        const auto & pose = poses.at(id);
        std::string colour = extractColour(id);
        bottles.push_back({pose.position.x, pose.position.y,
                           pose.position.z + BODY_HEIGHT, id, colour});
        RCLCPP_INFO(logger, "[discovery] '%s' colour='%s' at (%.3f,%.3f) z=%.3f",
            id.c_str(), colour.c_str(),
            pose.position.x, pose.position.y, pose.position.z + BODY_HEIGHT);
    }
    std::sort(bottles.begin(), bottles.end(),
        [](const BottlePosition & a, const BottlePosition & b) { return a.id < b.id; });
    RCLCPP_INFO(logger, "[discovery] %zu bottles total", bottles.size());
    return bottles;
}

static std::vector<PlacePosition> buildPlaceGrid(std::size_t count, const CrateConfig & crate)
{
    constexpr double spacing = 0.06;
    const double offsets[4][2] = {{-0.5,-0.5},{-0.5,0.5},{0.5,-0.5},{0.5,0.5}};
    std::vector<PlacePosition> grid;
    for (std::size_t i = 0; i < std::min(count, std::size_t(4)); i++)
        grid.push_back({
            crate.cx + offsets[i][0] * spacing,
            crate.cy + offsets[i][1] * spacing,
            crate.pz,
            crate.label + "_slot_" + std::to_string(i),
            crate.label
        });
    return grid;
}

// ===========================================================================
// MOTION EXECUTOR
// ===========================================================================
struct MotionExecutor {
    moveit::planning_interface::MoveGroupInterface & arm;
    std::atomic<int> & cmd;
    const rclcpp::Logger & logger;

    bool interrupted() const { return cmd == CMD_STOP || cmd == CMD_HOME; }

    bool isPathOk(const moveit::planning_interface::MoveGroupInterface::Plan & plan,
        const std::vector<double> & goal, double max_ratio) const
    {
        const auto & pts = plan.trajectory_.joint_trajectory.points;
        if (pts.size() < 2) return true;
        const size_t nj = pts[0].positions.size();
        for (size_t pt = 1; pt < pts.size(); pt++)
            for (size_t j = 0; j < nj; j++)
                if (std::abs(pts[pt].positions[j] - pts[pt-1].positions[j]) > MAX_SINGLE_JOINT_JUMP)
                    return false;
        double travel = 0, direct = 0;
        for (size_t pt = 1; pt < pts.size(); pt++)
            for (size_t j = 0; j < nj; j++)
                travel += std::abs(pts[pt].positions[j] - pts[pt-1].positions[j]);
        for (size_t j = 0; j < nj && j < goal.size(); j++)
            direct += std::abs(normaliseAngle(goal[j] - pts.front().positions[j]));
        return direct < 0.01 || (travel / direct) <= max_ratio;
    }

    bool jointMove(const std::vector<double> & joints, const std::string & label,
        int attempts = 3, int settle_ms = 120,
        double max_ratio = DEFAULT_PATH_RATIO, double plan_time = -1.0)
    {
        arm.clearPathConstraints();
        const double orig = arm.getPlanningTime();
        if (plan_time > 0) arm.setPlanningTime(plan_time);
        bool ok = false;
        for (int i = 1; i <= attempts && !interrupted(); i++) {
            rclcpp::sleep_for(std::chrono::milliseconds(settle_ms));
            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(joints);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(logger, "[%s] plan fail %d/%d", label.c_str(), i, attempts);
                continue;
            }
            if (!isPathOk(plan, joints, max_ratio)) continue;
            arm.execute(plan);
            if (interrupted()) break;
            RCLCPP_INFO(logger, "[%s] done", label.c_str());
            ok = true; break;
        }
        if (plan_time > 0) arm.setPlanningTime(orig);
        if (!ok && !interrupted())
            RCLCPP_ERROR(logger, "[%s] FAILED after %d attempts", label.c_str(), attempts);
        return ok;
    }

    bool cartesianMove(const geometry_msgs::msg::Pose & target, const std::string & label,
        double threshold = CARTESIAN_FRACTION, int attempts = 4)
    {
        arm.clearPathConstraints();
        for (int i = 1; i <= attempts && !interrupted(); i++) {
            arm.setStartStateToCurrentState();
            moveit_msgs::msg::RobotTrajectory traj;
            double frac = arm.computeCartesianPath({target}, CARTESIAN_STEP, 0.0, traj);
            if (frac < threshold) {
                RCLCPP_WARN(logger, "[%s] %.0f%% attempt %d/%d",
                    label.c_str(), frac*100, i, attempts);
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                continue;
            }
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = traj;
            arm.execute(plan);
            if (interrupted()) return false;
            RCLCPP_INFO(logger, "[%s] done (%.0f%%)", label.c_str(), frac*100);
            return true;
        }
        RCLCPP_ERROR(logger, "[%s] FAILED", label.c_str());
        return false;
    }
};

// ===========================================================================
// MAIN
// ===========================================================================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("move_to_pose", opts);

    auto spinner = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner->add_node(node);
    auto spin_thread = std::thread([&spinner]() { spinner->spin(); });
    auto shutdown_fn = [&] { spinner->cancel(); spin_thread.join(); rclcpp::shutdown(); };

    const auto logger = node->get_logger();

    std::atomic<int> current_cmd{CMD_NONE};
    std::mutex cmd_mutex;
    std::condition_variable cmd_cv;

    auto cmd_sub = node->create_subscription<std_msgs::msg::Int32>("system_command", 10,
        [&](const std_msgs::msg::Int32::SharedPtr msg) {
            RCLCPP_INFO(logger, "[GUI] cmd=%d", msg->data);
            current_cmd.store(msg->data);
            cmd_cv.notify_all();
        });

    // -----------------------------------------------------------------------
    // TF2 — camera_depth_optical_frame -> world
    // -----------------------------------------------------------------------
    auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    std::mutex camera_point_mutex;
    std::optional<BottlePosition> camera_bottle;
    std::atomic<bool> new_camera_point{false};

    auto camera_sub = node->create_subscription<geometry_msgs::msg::Point>(
        "bottle_center_average", 10,
        [&](const geometry_msgs::msg::Point::SharedPtr msg) {
            if (msg->x == 0.0 && msg->y == 0.0 && msg->z == 0.0) return;

            geometry_msgs::msg::PointStamped point_in;
            point_in.header.frame_id = "camera_depth_optical_frame";
            point_in.header.stamp    = node->now();
            point_in.point           = *msg;

            try {
                auto point_out = tf_buffer->transform(
                    point_in, "world", tf2::durationFromSec(1.0));

                std::lock_guard<std::mutex> lock(camera_point_mutex);
                camera_bottle = BottlePosition{
                    point_out.point.x,
                    point_out.point.y,
                    point_out.point.z,
                    "camera_bottle_0",
                    "red"
                };
                new_camera_point.store(true);
                // Also immediately save to saved_camera_bottle
                saved_camera_bottle = *camera_bottle;
                RCLCPP_INFO(logger, "[camera] point saved at world: (%.3f, %.3f, %.3f)",
                    point_out.point.x, point_out.point.y, point_out.point.z);
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(logger, "[camera] TF transform failed: %s", ex.what());
            }
        });

    moveit::planning_interface::MoveGroupInterface arm(node, "ur_onrobot_manipulator");
    arm.setEndEffectorLink("gripper_tcp");
    arm.setPlanningTime(PLAN_TIME);
    arm.setNumPlanningAttempts(3);
    arm.setMaxVelocityScalingFactor(VELOCITY);
    arm.setMaxAccelerationScalingFactor(ACCELERATION);

    RCLCPP_INFO(logger, "Switching controller...");
    system("timeout 2 ros2 control switch_controllers "
           "--activate scaled_joint_trajectory_controller "
           "--deactivate joint_trajectory_controller "
           "--controller-manager /controller_manager 2>/dev/null");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10);
    auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
        "planning_scene", 10);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    moveit::planning_interface::PlanningSceneInterface psi;
    const std::string frame = arm.getPlanningFrame();
    MotionExecutor executor{arm, current_cmd, logger};

    std::atomic<bool> watchdog_running{true};
    std::thread watchdog([&]() {
        int last = CMD_NONE;
        while (watchdog_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            int c = current_cmd.load();
            if ((c == CMD_STOP || c == CMD_HOME) && c != last) { arm.stop(); last = c; }
            else if (c != CMD_STOP && c != CMD_HOME) last = c;
        }
    });

    RCLCPP_INFO(logger, "Waiting for robot state...");
    for (int i = 0; i < 20; i++) {
        if (arm.getCurrentState(2.0)) { RCLCPP_INFO(logger, "Robot state ready."); break; }
        RCLCPP_WARN(logger, "State not ready (%d/20)", i + 1);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    std::vector<std::pair<BottlePosition, PlacePosition>> valid_jobs;
    size_t job_index = 0;
    MotionStep resume_step = MotionStep::PRE_PICK;
    std::string held_bottle;
    bool has_object = false;

    // Saved bottle position from LOCATE command
    std::optional<BottlePosition> saved_camera_bottle;

    auto emergencyRelease = [&](const std::string & bottle_id) {
        setGripper(gripper_pub, OPEN_WIDTH, logger, "emergency open");
        arm.detachObject(bottle_id);
        removeObject(psi, scene_pub, bottle_id, frame, logger);
        has_object = false; held_bottle = "";
    };

    RCLCPP_INFO(logger, "=== Ready. Waiting for command... ===");

    while (rclcpp::ok()) {
        {
            std::unique_lock<std::mutex> lock(cmd_mutex);
            cmd_cv.wait(lock, [&]() {
                int c = current_cmd.load();
                return c==CMD_START||c==CMD_HOME||c==CMD_RESUME||c==CMD_STOP||
                       c==CMD_LOCATE||!rclcpp::ok();
            });
        }
        int cmd_now = current_cmd.load();

        // ── HOME ──────────────────────────────────────────────────────────
        if (cmd_now == CMD_HOME) {
            RCLCPP_INFO(logger, "=== HOME ===");
            if (has_object) emergencyRelease(held_bottle);
            valid_jobs.clear(); job_index = 0;
            resume_step = MotionStep::PRE_PICK;
            saved_camera_bottle.reset();
            current_cmd.store(CMD_NONE);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            executor.jointMove(HOME_JOINTS, "HOME");
            RCLCPP_INFO(logger, "=== Homed. Waiting... ===");
            continue;
        }

        // ── LOCATE ────────────────────────────────────────────────────────────
        if (cmd_now == CMD_LOCATE) {
            current_cmd.store(CMD_NONE);
            if (saved_camera_bottle.has_value()) {
                RCLCPP_INFO(logger, "=== LOCATE: bottle already saved at world (%.3f, %.3f, %.3f) ===",
                    saved_camera_bottle->x,
                    saved_camera_bottle->y,
                    saved_camera_bottle->z);
            } else {
                RCLCPP_INFO(logger, "=== LOCATE: no point yet, listening for 5 seconds ===");
                auto deadline = node->now() + rclcpp::Duration::from_seconds(5.0);
                while (rclcpp::ok() && !new_camera_point.load()) {
                    if (node->now() > deadline) {
                        RCLCPP_WARN(logger, "[LOCATE] No camera point received in 5 seconds.");
                        break;
                    }
                    if (current_cmd == CMD_STOP || current_cmd == CMD_HOME) break;
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }
                if (new_camera_point.load()) {
                    std::lock_guard<std::mutex> lock(camera_point_mutex);
                    saved_camera_bottle = *camera_bottle;
                    new_camera_point.store(false);
                    RCLCPP_INFO(logger, "=== LOCATE: bottle saved at world (%.3f, %.3f, %.3f) ===",
                        saved_camera_bottle->x,
                        saved_camera_bottle->y,
                        saved_camera_bottle->z);
                }
            }
            continue;
        }

        // ── START ─────────────────────────────────────────────────────────
        if (cmd_now == CMD_START) {
            RCLCPP_INFO(logger, "=== START: moving to LOCATE position ===");
            current_cmd.store(CMD_NONE);
            setGripper(gripper_pub, CLOSED_WIDTH, logger, "close at start");
            if (!executor.jointMove(LOCATE_JOINTS, "LOCATE")) {
                RCLCPP_ERROR(logger, "Failed to reach LOCATE position.");
                continue;
            }
            RCLCPP_INFO(logger, "=== At LOCATE. Waiting for Move to 2nd View... ===");
            {
                std::unique_lock<std::mutex> lock(cmd_mutex);
                cmd_cv.wait(lock, [&]() {
                    int c = current_cmd.load();
                    return c==CMD_VIEW2||c==CMD_HOME||c==CMD_STOP||!rclcpp::ok();
                });
            }
            if (current_cmd.load() != CMD_VIEW2) continue;
        }

        // ── VIEW2 ─────────────────────────────────────────────────────────
        if (cmd_now == CMD_START || current_cmd.load() == CMD_VIEW2) {
            RCLCPP_INFO(logger, "=== VIEW2: moving to 2nd position ===");
            current_cmd.store(CMD_NONE);
            if (!executor.jointMove(VIEW2_JOINTS, "VIEW2")) {
                RCLCPP_ERROR(logger, "Failed to reach VIEW2 position.");
                continue;
            }
            RCLCPP_INFO(logger, "=== At VIEW2. Waiting for Complete Mission... ===");
            {
                std::unique_lock<std::mutex> lock(cmd_mutex);
                cmd_cv.wait(lock, [&]() {
                    int c = current_cmd.load();
                    return c==CMD_MISSION||c==CMD_HOME||c==CMD_STOP||!rclcpp::ok();
                });
            }
            if (current_cmd.load() != CMD_MISSION) continue;
            cmd_now = CMD_MISSION;
        }

        // ── MISSION ───────────────────────────────────────────────────────
        if (cmd_now == CMD_MISSION) {
            RCLCPP_INFO(logger, "=== MISSION: building job from bottle position ===");
            current_cmd.store(CMD_NONE);

            BottlePosition cam_bottle;

            // Use saved position from LOCATE if available
            if (saved_camera_bottle.has_value()) {
                cam_bottle = *saved_camera_bottle;
                RCLCPP_INFO(logger,
                    "=== MISSION: using saved LOCATE position (%.3f, %.3f, %.3f) ===",
                    cam_bottle.x, cam_bottle.y, cam_bottle.z);
                saved_camera_bottle.reset(); // clear after use
            } else {
                // Fall back to waiting for a fresh camera point (30s timeout)
                RCLCPP_INFO(logger,
                    "=== MISSION: no saved position, waiting for camera point (30s) ===");
                new_camera_point.store(false);
                auto deadline = node->now() + rclcpp::Duration::from_seconds(30.0);
                while (rclcpp::ok() && !new_camera_point.load()) {
                    if (node->now() > deadline) {
                        RCLCPP_ERROR(logger, "Timeout waiting for camera bottle position.");
                        break;
                    }
                    if (current_cmd == CMD_STOP || current_cmd == CMD_HOME) break;
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }
                if (!new_camera_point.load()) continue;
                {
                    std::lock_guard<std::mutex> lock(camera_point_mutex);
                    cam_bottle = *camera_bottle;
                    new_camera_point.store(false);
                }
            }

            RCLCPP_INFO(logger,
                "=== MISSION: bottle at world (%.3f, %.3f, %.3f) colour='%s' ===",
                cam_bottle.x, cam_bottle.y, cam_bottle.z, cam_bottle.colour.c_str());

            auto known = psi.getKnownObjectNames();
            if (std::find(known.begin(), known.end(), "ground") == known.end())
                addBox(psi, "ground", frame, 0.0, 0.0, -0.025, 2.0, 2.0, 0.05);

            // Build job list from camera bottle
            std::vector<BottlePosition> bottles = {cam_bottle};
            auto [crate1_bottles, crate2_bottles] = sortBottlesByColour(bottles, logger);
            auto grid1 = buildPlaceGrid(crate1_bottles.size(), CRATE_1);
            auto grid2 = buildPlaceGrid(crate2_bottles.size(), CRATE_2);

            RCLCPP_INFO(logger, "[sort] %s: %zu  |  %s: %zu",
                CRATE_1.label.c_str(), crate1_bottles.size(),
                CRATE_2.label.c_str(), crate2_bottles.size());

            valid_jobs.clear();

            for (size_t i = 0; i < grid1.size(); i++) {
                if (computeConfig(crate1_bottles[i], grid1[i], arm, logger))
                    valid_jobs.push_back({crate1_bottles[i], grid1[i]});
                else
                    RCLCPP_WARN(logger, "IK failed for camera bottle (crate1) -- skipping");
            }
            for (size_t i = 0; i < grid2.size(); i++) {
                if (computeConfig(crate2_bottles[i], grid2[i], arm, logger))
                    valid_jobs.push_back({crate2_bottles[i], grid2[i]});
                else
                    RCLCPP_WARN(logger, "IK failed for camera bottle (crate2) -- skipping");
            }

            if (valid_jobs.empty()) { RCLCPP_ERROR(logger, "No valid jobs."); continue; }

            RCLCPP_INFO(logger, "=== %zu total jobs — starting ===", valid_jobs.size());
            rclcpp::sleep_for(std::chrono::seconds(2));
            if (!executor.jointMove(HOME_JOINTS, "HOME before mission")) continue;
            job_index = 0; resume_step = MotionStep::PRE_PICK;
        }

        // ── RESUME ────────────────────────────────────────────────────────
        if (cmd_now == CMD_RESUME) {
            if (valid_jobs.empty()) {
                RCLCPP_WARN(logger, "Resume: no jobs loaded.");
                current_cmd.store(CMD_NONE); continue;
            }
            RCLCPP_INFO(logger, "=== RESUME job %zu step %s ===",
                job_index, stepName(resume_step).c_str());
            current_cmd.store(CMD_NONE);
        }

        // ── INNER MOTION LOOP ─────────────────────────────────────────────
        while (rclcpp::ok() && job_index < valid_jobs.size()) {
            int c = current_cmd.load();
            if (c == CMD_STOP) {
                RCLCPP_INFO(logger, "STOP at job %zu step %s",
                    job_index, stepName(resume_step).c_str());
                std::unique_lock<std::mutex> lock(cmd_mutex);
                cmd_cv.wait(lock, [&]() {
                    int cc = current_cmd.load();
                    return cc==CMD_RESUME||cc==CMD_HOME||!rclcpp::ok();
                });
                break;
            }
            if (c == CMD_HOME) break;

            const auto & [bottle, place] = valid_jobs[job_index];
            RCLCPP_INFO(logger, "=== Job %zu/%zu '%s'->'%s' from %s ===",
                job_index+1, valid_jobs.size(),
                bottle.id.c_str(), place.id.c_str(), stepName(resume_step).c_str());

            auto cfg = computeConfig(bottle, place, arm, logger);
            if (!cfg) {
                RCLCPP_WARN(logger, "Fresh IK failed -- skipping '%s'", bottle.id.c_str());
                job_index++; resume_step = MotionStep::PRE_PICK; continue;
            }

            auto interrupted = [&]() { return current_cmd==CMD_STOP||current_cmd==CMD_HOME; };
            auto skipJob = [&]() { job_index++; resume_step = MotionStep::PRE_PICK; };

            removeObject(psi, scene_pub, bottle.id, frame, logger);

            // PRE_PICK
            if (resume_step == MotionStep::PRE_PICK) {
                if (!executor.jointMove(cfg->pre_pick.joints, "PRE_PICK")) {
                    if (interrupted()) break;
                    executor.jointMove(HOME_JOINTS, "retreat HOME");
                    skipJob(); continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                setGripper(gripper_pub, OPEN_WIDTH, logger, "open");
                resume_step = MotionStep::PICK;
            }
            if (interrupted()) break;

            // PICK
            if (resume_step == MotionStep::PICK) {
                if (!executor.cartesianMove(cfg->pick.pose, "PICK", 0.35)) {
                    if (interrupted()) break;
                    executor.jointMove(cfg->pre_pick.joints, "retreat PRE_PICK");
                    executor.jointMove(HOME_JOINTS, "retreat HOME");
                    skipJob(); continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                setGripper(gripper_pub, CLOSED_WIDTH, logger, "close");
                arm.attachObject(bottle.id, "gripper_tcp", {
                    "left_inner_finger","left_finger_tip","right_inner_finger","right_finger_tip",
                    "left_inner_knuckle","right_inner_knuckle","left_outer_knuckle",
                    "right_outer_knuckle","gripper_tcp"});
                has_object = true; held_bottle = bottle.id;
                RCLCPP_INFO(logger, "Attached '%s'", bottle.id.c_str());
                resume_step = MotionStep::LIFT;
            }
            if (interrupted()) break;

            // LIFT
            if (resume_step == MotionStep::LIFT) {
                auto lift_target = arm.getCurrentPose().pose;
                lift_target.position.z = cfg->lift_target_z;
                if (!executor.cartesianMove(lift_target, "LIFT")) {
                    if (interrupted()) break;
                    emergencyRelease(bottle.id);
                    skipJob(); continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                resume_step = MotionStep::CARRY;
            }
            if (interrupted()) break;

            // CARRY
            if (resume_step == MotionStep::CARRY) {
                if (!executor.jointMove(cfg->carry.joints, "CARRY",
                        3, 120, CROSS_FAMILY_PATH_RATIO)) {
                    if (interrupted()) break;
                    emergencyRelease(bottle.id);
                    executor.jointMove(HOME_JOINTS, "retreat HOME");
                    skipJob(); continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(200 + CARRY_SETTLE_MS));
                resume_step = MotionStep::PRE_PLACE;
            }
            if (interrupted()) break;

            // PRE_PLACE
            if (resume_step == MotionStep::PRE_PLACE) {
                if (!executor.jointMove(cfg->pre_place.joints, "PRE_PLACE",
                        3, 250, CROSS_FAMILY_PATH_RATIO, PLAN_TIME_LONG)) {
                    if (interrupted()) break;
                    emergencyRelease(bottle.id);
                    executor.jointMove(cfg->carry.joints, "retreat CARRY");
                    executor.jointMove(HOME_JOINTS, "retreat HOME");
                    skipJob(); continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                resume_step = MotionStep::LOWER;
            }
            if (interrupted()) break;

            // LOWER
            if (resume_step == MotionStep::LOWER) {
                auto lower_target = arm.getCurrentPose().pose;
                lower_target.position.z = cfg->place_z;
                if (!executor.cartesianMove(lower_target, "LOWER", CARTESIAN_FRAC_SOFT))
                    RCLCPP_WARN(logger, "Lower incomplete -- releasing anyway.");
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                resume_step = MotionStep::RELEASE;
            }
            if (interrupted()) break;

            // RELEASE
            if (resume_step == MotionStep::RELEASE) {
                setGripper(gripper_pub, OPEN_WIDTH, logger, "open");
                arm.detachObject(bottle.id);
                has_object = false; held_bottle = "";
                removeObject(psi, scene_pub, bottle.id, frame, logger);
                setGripper(gripper_pub, CLOSED_WIDTH, logger, "close transit");
                resume_step = MotionStep::POST_PLACE;
            }
            if (interrupted()) break;

            // POST_PLACE
            if (resume_step == MotionStep::POST_PLACE) {
                executor.jointMove(cfg->post_place.joints, "POST_PLACE");
                resume_step = MotionStep::RETURN_CARRY;
            }
            if (interrupted()) break;

            // RETURN_CARRY
            if (resume_step == MotionStep::RETURN_CARRY) {
                executor.jointMove(cfg->pre_place.joints, "return PRE_PLACE");
                executor.jointMove(cfg->carry.joints,     "return CARRY");
            }

            RCLCPP_INFO(logger, "=== Job %zu complete ===", job_index + 1);
            skipJob();
        }

        if (job_index >= valid_jobs.size() && !valid_jobs.empty()
            && current_cmd != CMD_STOP && current_cmd != CMD_HOME) {
            RCLCPP_INFO(logger, "All jobs complete.");
            executor.jointMove(HOME_JOINTS, "final HOME");
            valid_jobs.clear(); job_index = 0;
            resume_step = MotionStep::PRE_PICK;
            current_cmd.store(CMD_NONE);
        }
    }

    watchdog_running = false;
    watchdog.join();
    shutdown_fn();
    return 0;
}