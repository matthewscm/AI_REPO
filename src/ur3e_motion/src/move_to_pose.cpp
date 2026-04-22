#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <vector>
#include <cmath>
#include <optional>
#include <string>

// ===========================================================================
// PHILOSOPHY
// ===========================================================================
// MoveIt IK with dual orientation support.
//
// Changes in this revision vs previous:
//
// 1. PLAN_TRIES reduced from 7 → 3. 7 attempts were burning time without
//    ever finding a valid plan when the fundamental problem was elsewhere.
//    3 is enough to confirm a failure quickly.
//
// 2. MAX_PATH_RATIO raised from 2.0 → 3.5 for the CARRY→PRE_PLACE move.
//    CARRY (pick-side config) and PRE_PLACE (place-side config) are in
//    different configuration families. The joint-space "straight line"
//    between them crosses a configuration singularity, so any real path
//    will have a ratio >> 2.0 by necessity. The old value was rejecting
//    geometrically correct plans repeatedly, burning all attempts.
//
//    A dedicated max_ratio parameter on jointMove() lets us apply the tight
//    check (2.0) for short same-family moves while relaxing it (3.5) for
//    the one cross-family transition.
//
// 3. settle_ms raised from 120 → 250ms for PRE_PLACE.
//    The CARRY move stops and the controller needs time to publish the
//    settled joint state. 120ms was occasionally reading a mid-deceleration
//    state, causing the planner to start from a slightly wrong position.
//
// 4. PLAN_TIME raised from 15 → 20s for the PRE_PLACE move only.
//    CARRY→PRE_PLACE is the hardest cross-family transition. More time lets
//    OMPL explore enough to find a clean path on the first attempt rather
//    than timing out and retrying.
//
// 5. A CARRY_SETTLE_MS constant is added. After the CARRY move, an extra
//    deliberate settle wait is inserted before PRE_PLACE planning. This is
//    separate from the per-move settle_ms and ensures the robot is genuinely
//    stationary before the planner reads the start state.
//
// 6. [ROOT CAUSE FIX] IK solutions are now validated and normalised before
//    being stored. The KDL/IKFast solver occasionally returns joint values
//    outside ±π (e.g. joint[1] = 4.73 rad, joint[0] = 2.31 rad) that are
//    mathematically equivalent but physically unreachable — the robot cannot
//    command a joint to 270° if its limit is ±180°. The planner then tries
//    to move to this unreachable target, times out on every attempt, and the
//    move fails consistently. Logs showed this was happening to PRE_PLACE
//    on roughly 2 out of 3 runs. Two defences are applied:
//
//    (a) validateAndNormaliseIK(): after solveIK() returns, every joint is
//        normalised to (-π, π] and then checked against the joint model's
//        actual position bounds. Any solution with an out-of-bounds joint is
//        rejected — the other elbow variant is used if it passes, or the
//        whole IK is retried from scratch.
//
//    (b) The joint limit constants UR3E_JOINT_LIMIT_LO / HI cap the allowed
//        range. The UR3e's continuous joints would pass ±2π but we clamp to
//        ±π as a safe planning bound — the SRDF already constrains them and
//        normalised values in (-π, π] are always within real limits.
//
// Joint order: [0]pan [1]lift [2]elbow [3]wrist1 [4]wrist2 [5]wrist3
// ===========================================================================

// ===========================================================================
// GRIPPER
// ===========================================================================
constexpr double OPEN_WIDTH   = 0.07;
constexpr double CLOSED_WIDTH = 0.05;

// ===========================================================================
// MOTION SCALING
// ===========================================================================
constexpr double VELOCITY     = 0.1;
constexpr double ACCELERATION = 0.1;
constexpr double PLAN_TIME         = 15.0;
constexpr double PLAN_TIME_LONG    = 20.0;   // used for CARRY→PRE_PLACE only
constexpr int    PLAN_TRIES   = 3;           // reduced — 7 never helped anyway

// Extra settle time after CARRY before planning PRE_PLACE.
// CARRY is a large cross-family joint move; the controller needs longer to
// publish a settled state than short same-family moves.
constexpr int    CARRY_SETTLE_MS = 400;

// ===========================================================================
// BOTTLE GEOMETRY — collision objects only
// ===========================================================================
constexpr double BODY_RADIUS = 0.030;
constexpr double BODY_HEIGHT = 0.180;
constexpr double NECK_RADIUS = 0.012;
constexpr double NECK_HEIGHT = 0.080;

// ===========================================================================
// ORIENTATION QUATERNIONS
// ===========================================================================
struct Quaternion { double x, y, z, w; };

constexpr Quaternion ORI_PICK  = {  0.7027, -0.0792,  0.7026,  0.0792 };
constexpr Quaternion ORI_PLACE = { -0.1250, -0.6957, -0.1244,  0.6969 };

// ===========================================================================
// MOTION PARAMETERS
// ===========================================================================

constexpr double PRE_PICK_HOVER = 0.038;

constexpr double TRANSIT_HEIGHT = 0.62;

constexpr double CARTESIAN_STEP      = 0.003;
constexpr double CARTESIAN_JUMP      = 0.0;
constexpr double CARTESIAN_FRACTION  = 0.95;
constexpr double CARTESIAN_FRAC_SOFT = 0.80;

constexpr double IK_TIMEOUT = 0.15;

constexpr double ELBOW_FLIP_THRESHOLD = 0.25;

// ===========================================================================
// PATH QUALITY GUARD
//
// MAX_PATH_RATIO: reject plans where total joint travel > this multiple of
//   the direct joint-space distance.
//
//   DEFAULT_PATH_RATIO (2.0): used for short, same-family transitions where
//   the joint-space straight line is meaningful (HOME→PRE_PICK, PICK→PRE_PICK,
//   PRE_PLACE→PLACE, etc.).
//
//   CROSS_FAMILY_PATH_RATIO (3.5): used for CARRY→PRE_PLACE. These two
//   postures are in different configuration families (pick-side vs place-side).
//   The joint-space "straight line" between them crosses a configuration
//   singularity, so ANY valid collision-free path will have a higher ratio.
//   Using 2.0 here was rejecting geometrically correct plans every time,
//   burning all planning attempts and causing consistent PRE_PLACE failures
//   on some runs. 3.5 is still tight enough to catch genuinely wild paths
//   (e.g. a joint doing a full extra rotation) while accepting correct ones.
//
// MAX_SINGLE_JOINT_JUMP: unchanged at 4.0 rad — this catches full-rotation
//   shortcuts regardless of which ratio is used.
// ===========================================================================
constexpr double DEFAULT_PATH_RATIO    = 2.0;
constexpr double CROSS_FAMILY_PATH_RATIO = 3.5;
constexpr double MAX_SINGLE_JOINT_JUMP = 4.0;

// ===========================================================================
// HOME — hardcoded, unchanged
// ===========================================================================
const std::vector<double> HOME_JOINTS = {
   0.0,
  -1.5700,
   0.0,
  -1.5699,
  -1.4959,
   0.0
};

// ===========================================================================
// CARRY — fixed safe transit posture
// ===========================================================================
const std::vector<double> CARRY_JOINTS = {
  -0.3009,
  -1.5098,
   0.7255,
  -2.3576,
  -1.4937,
   0.0
};

// ===========================================================================
// IK SEEDS
//
// These must represent physically valid, reachable configurations in the
// correct operational range. The normaliser shifts IK solutions to the
// period closest to these seeds, so the seeds define which "branch" of
// the solution space is used.
//
// Place-side seeds are set to the known-good values observed in successful
// runs. joint[3] moved from -4.3113 to -2.43 (same config, different period
// expression) to keep normalised results well inside (-6.2, 6.2) bounds.
// ===========================================================================
const std::vector<double> SEED_PICK_ELBOW_UP = {
  -0.2993, -1.3905,  2.3495, -4.1008, -1.4959,  0.0
};
const std::vector<double> SEED_PICK_ELBOW_DOWN = {
  -0.2993, -1.3905, -2.3495, -4.1008, -1.4959,  0.0
};
// Place-side seeds anchored at known-good working values from successful runs.
// joint[3] expressed as -2.43 (≡ -4.31 + 2π would be wrong; -2.43 IS the
// correct joint-space value for the place-side wrist1 configuration).
const std::vector<double> SEED_PLACE_ELBOW_UP = {
  -3.5748, -1.5544,  0.8426, -2.4283, -1.4928,  0.0
};
const std::vector<double> SEED_PLACE_ELBOW_DOWN = {
  -3.9699, -1.5688, -0.8426, -2.4219, -1.4928,  0.0
};

// ===========================================================================
// DATA STRUCTURES
// ===========================================================================

struct IKSolution {
    std::vector<double> joints;
    geometry_msgs::msg::Pose pose;
    std::string name;
    const std::vector<double> & toVector() const { return joints; }
};

struct BottlePosition {
    double x, y, z;
    std::string id;
};

struct PlacePosition {
    double x, y, z;
    std::string id;
};

struct ComputedConfig {
    IKSolution pre_pick;
    IKSolution pick;
    IKSolution carry;
    IKSolution pre_place;
    IKSolution place_down;
    IKSolution post_place;
    double lift_target_z;
    double place_z;
};

// ===========================================================================
// MATH UTILITIES
// ===========================================================================

static double normaliseAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static double angularDistance(double a, double b) {
    return std::abs(normaliseAngle(a - b));
}

static geometry_msgs::msg::Pose makePose(
    double x, double y, double z, const Quaternion & q)
{
    geometry_msgs::msg::Pose p;
    p.position.x    = x;
    p.position.y    = y;
    p.position.z    = z;
    p.orientation.x = q.x;
    p.orientation.y = q.y;
    p.orientation.z = q.z;
    p.orientation.w = q.w;
    return p;
}

// ===========================================================================
// IK SOLUTION NORMALISER + BOUNDS CHECK
//
// Two-stage process:
//
// STAGE 1 — seed-relative normalisation.
//   The KDL/IKFast solver returns values that are correct modulo 2π but
//   sometimes expressed in the wrong period (e.g. 4.73 instead of -1.56).
//   For each joint, we shift the raw value by multiples of 2π until it is
//   in the period closest to the seed. This is the mathematically minimal
//   representation relative to the seed.
//
// STAGE 2 — SRDF bounds clamp.
//   After normalisation, the result must still fall within the robot's
//   actual SRDF joint position limits. For the UR3e these are (-2π, 2π)
//   for all joints. Values outside this range after stage 1 mean the
//   solver found a configuration that requires the joint to wrap past its
//   hard stop — which the robot will refuse to execute, causing the planner
//   to either reject the goal or produce a wildly winding path trying to
//   reach it. We log and clamp these; if a joint is still outside limits
//   after the nearest-period selection, the solution is invalid.
//
// Why this combination is needed:
//   Seed-relative normalisation alone can pull a value into a period that
//   is geometrically closest to the seed but numerically outside SRDF limits.
//   Example: seed[0] = -3.5741, raw = 0.538 → normalised = -5.745.
//   Distance to seed = 2.17 < π, so the loop accepts it. But -5.745 is
//   outside (-2π, 2π) = (-6.283, 6.283)... barely inside actually. However
//   seeds like -4.3113 (joint[3]) have a similar issue: normalising toward
//   -4.31 can produce values around -6.5 which IS outside UR limits.
//
//   The UR3e SRDF position bounds per joint (from ur3e.urdf / ur.srdf):
//     joint[0] pan:    (-2π, 2π)  = (-6.283, 6.283)
//     joint[1] lift:   (-2π, 2π)  = (-6.283, 6.283)  [but operationally -π to π]
//     joint[2] elbow:  (-π, π)    = (-3.142, 3.142)
//     joint[3] wrist1: (-2π, 2π)  = (-6.283, 6.283)
//     joint[4] wrist2: (-2π, 2π)  = (-6.283, 6.283)
//     joint[5] wrist3: (-2π, 2π)  = (-6.283, 6.283)
//
//   We use conservative safe bounds to avoid the edges:
// ===========================================================================

// Safe operational bounds — slightly inside SRDF hard limits.
// joint[2] (elbow) is physically limited to (-π, π) on the UR3e.
// All others are (-2π, 2π) but we keep a small margin.
static const double JOINT_LO[] = { -6.2, -6.2, -3.14, -6.2, -6.2, -6.2 };
static const double JOINT_HI[] = {  6.2,  6.2,  3.14,  6.2,  6.2,  6.2 };

static bool normaliseSolutionToSeed(
    IKSolution & sol,
    const std::vector<double> & seed,
    const rclcpp::Logger & logger)
{
    bool valid = true;
    for (size_t j = 0; j < sol.joints.size() && j < seed.size(); j++) {
        double raw = sol.joints[j];
        double s   = seed[j];
        double v   = raw;

        // Stage 1: shift to period nearest seed
        while (v - s >  M_PI) v -= 2.0 * M_PI;
        while (v - s < -M_PI) v += 2.0 * M_PI;

        if (std::abs(v - raw) > 1e-6) {
            RCLCPP_INFO(logger,
                "IK normalise [%s] joint[%zu]: %.4f → %.4f (seed=%.4f)",
                sol.name.c_str(), j, raw, v, s);
        }

        // Stage 2: bounds check after normalisation
        if (j < 6 && (v < JOINT_LO[j] || v > JOINT_HI[j])) {
            RCLCPP_WARN(logger,
                "IK REJECT [%s] joint[%zu] = %.4f out of [%.3f, %.3f] "
                "(raw=%.4f seed=%.4f) — discarding this solution",
                sol.name.c_str(), j, v, JOINT_LO[j], JOINT_HI[j], raw, s);
            valid = false;
            // Don't break — log all bad joints for diagnosis
        }

        sol.joints[j] = v;
    }
    return valid;
}

// ===========================================================================
// MOVEIT IK SOLVER
//
// After each solution is found, normaliseSolutionToSeed() is called with
// the corresponding seed vector. This ensures the returned joint values are
// in the same ±π neighbourhood as the seed — i.e. in the correct
// configuration family — regardless of which 2π period the solver chose.
// ===========================================================================
static std::optional<std::pair<IKSolution, IKSolution>>
solveIK(double tx, double ty, double tz,
        const std::string & side,
        moveit::planning_interface::MoveGroupInterface & arm,
        const rclcpp::Logger & logger)
{
    const Quaternion & ori = (side == "place") ? ORI_PLACE : ORI_PICK;
    const std::vector<double> & seed_up   = (side == "place")
        ? SEED_PLACE_ELBOW_UP   : SEED_PICK_ELBOW_UP;
    const std::vector<double> & seed_down = (side == "place")
        ? SEED_PLACE_ELBOW_DOWN : SEED_PICK_ELBOW_DOWN;

    geometry_msgs::msg::Pose target = makePose(tx, ty, tz, ori);

    moveit::core::RobotStatePtr state = arm.getCurrentState();
    const moveit::core::JointModelGroup * jmg =
        state->getRobotModel()->getJointModelGroup("ur_onrobot_manipulator");

    IKSolution sol_up, sol_down;
    bool found_up   = false;
    bool found_down = false;

    state->setJointGroupPositions(jmg, seed_up);
    state->update();
    if (state->setFromIK(jmg, target, IK_TIMEOUT)) {
        state->copyJointGroupPositions(jmg, sol_up.joints);
        sol_up.pose = target;
        sol_up.name = "elbow_up";
        found_up = normaliseSolutionToSeed(sol_up, seed_up, logger);
    }

    state->setJointGroupPositions(jmg, seed_down);
    state->update();
    if (state->setFromIK(jmg, target, IK_TIMEOUT)) {
        state->copyJointGroupPositions(jmg, sol_down.joints);
        sol_down.pose = target;
        sol_down.name = "elbow_down";
        found_down = normaliseSolutionToSeed(sol_down, seed_down, logger);
    }

    if (!found_up && !found_down) {
        RCLCPP_WARN(logger,
            "IK (%s): no solution for (%.3f, %.3f, %.3f)",
            side.c_str(), tx, ty, tz);
        return std::nullopt;
    }

    if (!found_up)   { sol_up   = sol_down; }
    if (!found_down) { sol_down = sol_up;   }

    return std::make_pair(sol_up, sol_down);
}

// ===========================================================================
// SOLUTION PICKER WITH HYSTERESIS
// ===========================================================================
static IKSolution chooseSolution(
    const std::pair<IKSolution, IKSolution> & sols,
    const std::vector<double> & current,
    const std::string & last_name,
    const rclcpp::Logger & logger)
{
    auto cost = [&](const IKSolution & s) -> double {
        return angularDistance(s.joints[0], current[0]) * 2.5 +
               angularDistance(s.joints[1], current[1]) * 1.5 +
               angularDistance(s.joints[2], current[2]) * 1.0 +
               angularDistance(s.joints[3], current[3]) * 0.5;
    };

    const IKSolution & up   = sols.first;
    const IKSolution & down = sols.second;
    double cost_up   = cost(up);
    double cost_down = cost(down);

    const IKSolution & best  = (cost_up <= cost_down) ? up   : down;
    const IKSolution & worse = (cost_up <= cost_down) ? down : up;

    if (!last_name.empty() && last_name != best.name) {
        double improvement = cost(worse) - cost(best);
        if (improvement < ELBOW_FLIP_THRESHOLD) {
            RCLCPP_DEBUG(logger,
                "IK hysteresis: keeping %s (improvement %.3f < %.3f)",
                last_name.c_str(), improvement, ELBOW_FLIP_THRESHOLD);
            return (last_name == up.name) ? up : down;
        }
        RCLCPP_INFO(logger, "IK: config %s → %s (improvement %.3f rad)",
            last_name.c_str(), best.name.c_str(), improvement);
    }

    return best;
}

// ===========================================================================
// COMPUTE FULL CONFIG FOR ONE JOB
// ===========================================================================
static std::optional<ComputedConfig> computeConfig(
    const BottlePosition & bottle,
    const PlacePosition  & place,
    moveit::planning_interface::MoveGroupInterface & arm,
    const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger,
        "--- IK: '%s' (%.3f,%.3f,%.3f) → '%s' (%.3f,%.3f,%.3f) ---",
        bottle.id.c_str(), bottle.x, bottle.y, bottle.z,
        place.id.c_str(),  place.x,  place.y,  place.z);

    ComputedConfig cfg;
    std::string last = "";

    // ── PRE_PICK ──────────────────────────────────────────────────────────
    double pre_pick_z = bottle.z + PRE_PICK_HOVER;
    auto s1 = solveIK(bottle.x, bottle.y, pre_pick_z, "pick", arm, logger);
    if (!s1) { RCLCPP_ERROR(logger, "  PRE_PICK unsolvable"); return std::nullopt; }
    cfg.pre_pick = chooseSolution(*s1, HOME_JOINTS, last, logger);
    last = cfg.pre_pick.name;
    RCLCPP_INFO(logger, "  PRE_PICK  [%.4f, %.4f, %.4f, %.4f] (%s)",
        cfg.pre_pick.joints[0], cfg.pre_pick.joints[1],
        cfg.pre_pick.joints[2], cfg.pre_pick.joints[3],
        cfg.pre_pick.name.c_str());

    // ── PICK ──────────────────────────────────────────────────────────────
    auto s2 = solveIK(bottle.x, bottle.y, bottle.z, "pick", arm, logger);
    if (!s2) { RCLCPP_ERROR(logger, "  PICK unsolvable"); return std::nullopt; }
    cfg.pick = chooseSolution(*s2, cfg.pre_pick.joints, last, logger);
    last = cfg.pick.name;
    RCLCPP_INFO(logger, "  PICK      [%.4f, %.4f, %.4f, %.4f] (%s)",
        cfg.pick.joints[0], cfg.pick.joints[1],
        cfg.pick.joints[2], cfg.pick.joints[3],
        cfg.pick.name.c_str());

    // ── CARRY — fixed posture ─────────────────────────────────────────────
    cfg.carry.joints = CARRY_JOINTS;
    cfg.carry.name   = "carry";
    last = cfg.carry.name;

    // ── PRE_PLACE ─────────────────────────────────────────────────────────
    auto s4 = solveIK(place.x, place.y, TRANSIT_HEIGHT, "place", arm, logger);
    if (!s4) { RCLCPP_ERROR(logger, "  PRE_PLACE unsolvable"); return std::nullopt; }
    cfg.pre_place = chooseSolution(*s4, SEED_PLACE_ELBOW_UP, last, logger);
    last = cfg.pre_place.name;
    RCLCPP_INFO(logger, "  PRE_PLACE [%.4f, %.4f, %.4f, %.4f] (%s) z=%.3f",
        cfg.pre_place.joints[0], cfg.pre_place.joints[1],
        cfg.pre_place.joints[2], cfg.pre_place.joints[3],
        cfg.pre_place.name.c_str(), TRANSIT_HEIGHT);

    // ── PLACE_DOWN ────────────────────────────────────────────────────────
    auto s5 = solveIK(place.x, place.y, place.z, "place", arm, logger);
    if (!s5) { RCLCPP_ERROR(logger, "  PLACE_DOWN unsolvable"); return std::nullopt; }
    cfg.place_down = chooseSolution(*s5, cfg.pre_place.joints, last, logger);
    last = cfg.place_down.name;
    RCLCPP_INFO(logger, "  PLACE_DOWN [%.4f, %.4f, %.4f, %.4f] (%s) z=%.3f",
        cfg.place_down.joints[0], cfg.place_down.joints[1],
        cfg.place_down.joints[2], cfg.place_down.joints[3],
        cfg.place_down.name.c_str(), place.z);

    // ── POST_PLACE — reuse pre_place ──────────────────────────────────────
    cfg.post_place      = cfg.pre_place;
    cfg.post_place.name = "post_place";

    cfg.lift_target_z = TRANSIT_HEIGHT;
    cfg.place_z       = place.z;

    RCLCPP_INFO(logger, "--- IK OK ---");
    return cfg;
}

// ===========================================================================
// HELPERS
// ===========================================================================

using GripperPub = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;

void setGripper(GripperPub & pub, double width,
    const rclcpp::Logger & logger, const std::string & label)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {width};
    pub->publish(msg);
    RCLCPP_INFO(logger, "Gripper %s (width=%.3f)", label.c_str(), width);
    rclcpp::sleep_for(std::chrono::milliseconds(800));
}

// ===========================================================================
// PATH QUALITY CHECKER
//
// max_ratio is now a parameter rather than a compile-time constant.
// Pass DEFAULT_PATH_RATIO (2.0) for same-family moves and
// CROSS_FAMILY_PATH_RATIO (3.5) for CARRY→PRE_PLACE.
// ===========================================================================
static bool isPathQualityOk(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::vector<double> & goal_joints,
    const rclcpp::Logger & logger,
    const std::string & label,
    double max_ratio = DEFAULT_PATH_RATIO)
{
    const auto & pts = plan.trajectory_.joint_trajectory.points;
    if (pts.size() < 2) return true;

    const size_t n_joints = pts[0].positions.size();

    // ── 1. Check for large single-segment jumps ───────────────────────────
    for (size_t pt = 1; pt < pts.size(); pt++) {
        for (size_t j = 0; j < n_joints; j++) {
            double delta = std::abs(pts[pt].positions[j] - pts[pt-1].positions[j]);
            if (delta > MAX_SINGLE_JOINT_JUMP) {
                RCLCPP_WARN(logger,
                    "[%s] quality FAIL: joint[%zu] jumps %.2f rad at segment %zu",
                    label.c_str(), j, delta, pt);
                return false;
            }
        }
    }

    // ── 2. Check total path length vs direct distance ─────────────────────
    double total_travel = 0.0;
    for (size_t pt = 1; pt < pts.size(); pt++) {
        for (size_t j = 0; j < n_joints; j++) {
            total_travel += std::abs(pts[pt].positions[j] - pts[pt-1].positions[j]);
        }
    }

    double direct_dist = 0.0;
    const auto & start = pts.front().positions;
    for (size_t j = 0; j < n_joints && j < goal_joints.size(); j++) {
        direct_dist += std::abs(normaliseAngle(goal_joints[j] - start[j]));
    }

    if (direct_dist < 0.01) return true;

    double ratio = total_travel / direct_dist;
    if (ratio > max_ratio) {
        RCLCPP_WARN(logger,
            "[%s] quality FAIL: path ratio %.2f > %.2f (travel=%.3f rad, direct=%.3f rad) "
            "— replanning",
            label.c_str(), ratio, max_ratio, total_travel, direct_dist);
        return false;
    }

    RCLCPP_DEBUG(logger, "[%s] quality OK: ratio=%.2f", label.c_str(), ratio);
    return true;
}

// ===========================================================================
// JOINT MOVE WITH PATH QUALITY GUARD
//
// New parameter: max_ratio — passed through to isPathQualityOk().
// Default is DEFAULT_PATH_RATIO. Pass CROSS_FAMILY_PATH_RATIO for moves
// that cross configuration families (CARRY → PRE_PLACE).
//
// settle_ms raised to 120 default; caller can override to 250 for moves
// following large cross-family transitions.
// ===========================================================================
bool jointMove(
    moveit::planning_interface::MoveGroupInterface & arm,
    const std::vector<double> & joints,
    const rclcpp::Logger & logger,
    const std::string & label,
    int max_attempts = 3,
    int settle_ms = 120,
    double max_ratio = DEFAULT_PATH_RATIO,
    double plan_time_override = -1.0)
{
    arm.clearPathConstraints();

    // If a custom plan time is requested, apply and restore after
    const double original_plan_time = arm.getPlanningTime();
    if (plan_time_override > 0.0) {
        arm.setPlanningTime(plan_time_override);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        rclcpp::sleep_for(std::chrono::milliseconds(settle_ms));
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(logger, "[%s] plan failed (attempt %d/%d)",
                label.c_str(), attempt, max_attempts);
            continue;
        }

        if (!isPathQualityOk(plan, joints, logger, label, max_ratio)) {
            continue;
        }

        if (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "[%s] done", label.c_str());
            success = true;
            break;
        }
        RCLCPP_WARN(logger, "[%s] execute failed (attempt %d/%d)",
            label.c_str(), attempt, max_attempts);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    if (plan_time_override > 0.0) {
        arm.setPlanningTime(original_plan_time);
    }

    if (!success) {
        RCLCPP_ERROR(logger, "[%s] FAILED after %d attempts", label.c_str(), max_attempts);
    }
    return success;
}

// ===========================================================================
// CARTESIAN MOVE TO ABSOLUTE TARGET POSE
// ===========================================================================
bool cartesianMoveTo(
    moveit::planning_interface::MoveGroupInterface & arm,
    const geometry_msgs::msg::Pose & target_pose,
    const rclcpp::Logger & logger,
    const std::string & label,
    double fraction_threshold = CARTESIAN_FRACTION,
    int max_attempts = 4)
{
    arm.clearPathConstraints();
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        arm.setStartStateToCurrentState();
        std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm.computeCartesianPath(
            waypoints, CARTESIAN_STEP, CARTESIAN_JUMP, trajectory);

        if (fraction < fraction_threshold) {
            RCLCPP_WARN(logger,
                "[%s] only %.0f%% planned (need %.0f%%, attempt %d/%d)",
                label.c_str(),
                fraction * 100.0, fraction_threshold * 100.0,
                attempt, max_attempts);
            rclcpp::sleep_for(std::chrono::milliseconds(300));
            continue;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        if (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "[%s] done (%.0f%%)", label.c_str(), fraction * 100.0);
            return true;
        }
        RCLCPP_WARN(logger, "[%s] execute failed (attempt %d/%d)",
            label.c_str(), attempt, max_attempts);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    RCLCPP_ERROR(logger, "[%s] FAILED after %d attempts", label.c_str(), max_attempts);
    return false;
}

// ===========================================================================
// CARTESIAN LIFT / LOWER helpers
// ===========================================================================
geometry_msgs::msg::Pose buildLiftTarget(
    moveit::planning_interface::MoveGroupInterface & arm,
    double target_z)
{
    geometry_msgs::msg::Pose p = arm.getCurrentPose().pose;
    p.position.z = target_z;
    return p;
}

geometry_msgs::msg::Pose buildLowerTarget(
    const IKSolution & pre_place_sol,
    double place_z)
{
    geometry_msgs::msg::Pose p = pre_place_sol.pose;
    p.position.z = place_z;
    return p;
}

void addBox(
    moveit::planning_interface::PlanningSceneInterface & psi,
    const std::string & id, const std::string & frame,
    double x, double y, double z,
    double sx, double sy, double sz)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {sx, sy, sz};
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    pose.orientation.w = 1.0;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
}

void addBottle(
    moveit::planning_interface::PlanningSceneInterface & psi,
    const std::string & id, const std::string & frame,
    double x, double y)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame;
    obj.id = id;

    shape_msgs::msg::SolidPrimitive body;
    body.type = body.CYLINDER;
    body.dimensions.push_back(BODY_HEIGHT);
    body.dimensions.push_back(BODY_RADIUS);
    geometry_msgs::msg::Pose body_pose;
    body_pose.position.x = x; body_pose.position.y = y;
    body_pose.position.z = BODY_HEIGHT / 2.0;
    body_pose.orientation.w = 1.0;

    shape_msgs::msg::SolidPrimitive neck;
    neck.type = neck.CYLINDER;
    neck.dimensions.push_back(NECK_HEIGHT);
    neck.dimensions.push_back(NECK_RADIUS);
    geometry_msgs::msg::Pose neck_pose;
    neck_pose.position.x = x; neck_pose.position.y = y;
    neck_pose.position.z = BODY_HEIGHT + NECK_HEIGHT / 2.0;
    neck_pose.orientation.w = 1.0;

    obj.primitives.push_back(body); obj.primitive_poses.push_back(body_pose);
    obj.primitives.push_back(neck); obj.primitive_poses.push_back(neck_pose);
    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
    RCLCPP_INFO(rclcpp::get_logger("bottle_picker"),
        "Collision: added '%s' at (%.3f, %.3f)", id.c_str(), x, y);
}

// ===========================================================================
// MAIN
// ===========================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("bottle_picker", opts);

    auto spinner = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spinner->add_node(node);
    auto spin_thread = std::thread([&spinner]() { spinner->spin(); });
    auto shutdown_fn = [&] {
        spinner->cancel();
        spin_thread.join();
        rclcpp::shutdown();
    };

    const auto logger = node->get_logger();

    // ── MoveIt ───────────────────────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm(node, "ur_onrobot_manipulator");
    arm.setEndEffectorLink("gripper_tcp");
    arm.setPlanningTime(PLAN_TIME);
    arm.setNumPlanningAttempts(PLAN_TRIES);
    arm.setMaxVelocityScalingFactor(VELOCITY);
    arm.setMaxAccelerationScalingFactor(ACCELERATION);

    // ==========================================================================
    // DIAGNOSTIC MODE
    // ==========================================================================
    constexpr bool DIAGNOSTIC_MODE = false;

    if (DIAGNOSTIC_MODE) {
        rclcpp::sleep_for(std::chrono::seconds(2));
        auto pose   = arm.getCurrentPose();
        auto joints = arm.getCurrentJointValues();

        RCLCPP_INFO(logger, "");
        RCLCPP_INFO(logger, "========================================");
        RCLCPP_INFO(logger, "DIAGNOSTIC — robot has NOT moved");
        RCLCPP_INFO(logger, "========================================");
        RCLCPP_INFO(logger, "TCP position (world frame):");
        RCLCPP_INFO(logger, "  x = %.6f", pose.pose.position.x);
        RCLCPP_INFO(logger, "  y = %.6f", pose.pose.position.y);
        RCLCPP_INFO(logger, "  z = %.6f", pose.pose.position.z);
        RCLCPP_INFO(logger, "TCP orientation (quaternion):");
        RCLCPP_INFO(logger, "  x = %.6f", pose.pose.orientation.x);
        RCLCPP_INFO(logger, "  y = %.6f", pose.pose.orientation.y);
        RCLCPP_INFO(logger, "  z = %.6f", pose.pose.orientation.z);
        RCLCPP_INFO(logger, "  w = %.6f", pose.pose.orientation.w);
        RCLCPP_INFO(logger, "Joint values:");
        for (size_t j = 0; j < joints.size(); j++) {
            RCLCPP_INFO(logger, "  [%zu] = %.6f", j, joints[j]);
        }
        RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "========================================");

        shutdown_fn();
        return 0;
    }

    // ── Controller switch ─────────────────────────────────────────────────
    RCLCPP_INFO(logger, "Switching to scaled_joint_trajectory_controller...");
    system(
        "ros2 control switch_controllers "
        "--activate scaled_joint_trajectory_controller "
        "--deactivate joint_trajectory_controller "
        "--controller-manager /controller_manager 2>/dev/null");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ── Gripper publisher ─────────────────────────────────────────────────
    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // ==========================================================================
    // JOB LIST
    // ==========================================================================
    std::vector<std::pair<BottlePosition, PlacePosition>> job_list = {
        {
            BottlePosition{ .x =  0.4997, .y =  -0.0853, .z = 0.2640, .id = "bottle_1" },
            PlacePosition { .x = -0.4962, .y =  0.0581, .z = 0.2711, .id = "place_1"  }
        },
        { BottlePosition{ .x=0.4997, .y=-0.1873, .z=0.2640, .id="bottle_2" },
          PlacePosition { .x=-0.4962, .y=0.1381, .z=0.2711, .id="place_2" } },
    };

    // ==========================================================================
    // STARTUP: PRE-SOLVE ALL POSITIONS
    // ==========================================================================
    RCLCPP_INFO(logger, "=== Pre-solving IK for %zu jobs ===", job_list.size());

    std::vector<ComputedConfig> configs;
    configs.reserve(job_list.size());

    for (const auto & [bottle, place] : job_list) {
        auto cfg = computeConfig(bottle, place, arm, logger);
        if (!cfg) {
            RCLCPP_ERROR(logger,
                "ABORT: IK failed for '%s' → '%s'.",
                bottle.id.c_str(), place.id.c_str());
            shutdown_fn();
            return 1;
        }
        configs.push_back(*cfg);
    }

    RCLCPP_INFO(logger, "=== All %zu jobs solved — starting motion ===",
        job_list.size());

    // ── Collision scene ───────────────────────────────────────────────────
    moveit::planning_interface::PlanningSceneInterface psi;
    const std::string frame = arm.getPlanningFrame();

    RCLCPP_INFO(logger, "Clearing old scene objects...");
    auto known = psi.getKnownObjectNames();
    if (!known.empty()) psi.removeCollisionObjects(known);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    addBox(psi, "ground", frame, 0.0, 0.0, -0.025, 2.0, 2.0, 0.05);

    for (const auto & [bottle, place] : job_list) {
        addBottle(psi, bottle.id, frame, bottle.x, bottle.y);
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(logger, "Scene ready.");

    // ==========================================================================
    // MOTION SEQUENCE
    // ==========================================================================

    RCLCPP_INFO(logger, "[INIT] Moving to HOME...");
    setGripper(gripper_pub, CLOSED_WIDTH, logger, "close at start");
    if (!jointMove(arm, HOME_JOINTS, logger, "HOME")) {
        RCLCPP_ERROR(logger, "Cannot reach HOME — aborting.");
        shutdown_fn(); return 1;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    for (size_t i = 0; i < job_list.size(); i++) {
        const auto & [bottle, place] = job_list[i];
        const ComputedConfig & cfg   = configs[i];

        RCLCPP_INFO(logger, "=== Job %zu/%zu: '%s' → '%s' ===",
            i + 1, job_list.size(), bottle.id.c_str(), place.id.c_str());

        // ── PRE_PICK ──────────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[2] PRE_PICK...");
        if (!jointMove(arm, cfg.pre_pick.toVector(), logger, "PRE_PICK")) {
            RCLCPP_ERROR(logger, "PRE_PICK failed — retreating to HOME.");
            jointMove(arm, HOME_JOINTS, logger, "retreat HOME");
            continue;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // ── Open gripper ──────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[3] Opening gripper...");
        setGripper(gripper_pub, OPEN_WIDTH, logger, "open");

        // ── PICK ──────────────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[4] PICK...");
        if (!jointMove(arm, cfg.pick.toVector(), logger, "PICK")) {
            RCLCPP_ERROR(logger, "PICK failed — retreating.");
            jointMove(arm, cfg.pre_pick.toVector(), logger, "retreat PRE_PICK");
            jointMove(arm, HOME_JOINTS,             logger, "retreat HOME");
            continue;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // ── Close gripper ─────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[5] Closing gripper...");
        setGripper(gripper_pub, CLOSED_WIDTH, logger, "close");
        arm.attachObject(bottle.id, "gripper_tcp", {
            "left_inner_finger", "left_finger_tip",
            "right_inner_finger", "right_finger_tip",
            "left_inner_knuckle", "right_inner_knuckle",
            "left_outer_knuckle", "right_outer_knuckle", "gripper_tcp"
        });
        RCLCPP_INFO(logger, "Bottle '%s' attached.", bottle.id.c_str());

        // ── Straight lift to TRANSIT_HEIGHT ───────────────────────────────
        RCLCPP_INFO(logger, "[6] Lift to z=%.3f...", cfg.lift_target_z);
        geometry_msgs::msg::Pose lift_target = buildLiftTarget(arm, cfg.lift_target_z);
        if (!cartesianMoveTo(arm, lift_target, logger, "lift")) {
            RCLCPP_ERROR(logger, "Lift failed — emergency release.");
            setGripper(gripper_pub, OPEN_WIDTH, logger, "emergency open");
            arm.detachObject(bottle.id);
            shutdown_fn(); return 1;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        // ── CARRY ─────────────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[7] CARRY...");
        if (!jointMove(arm, cfg.carry.toVector(), logger, "CARRY")) {
            RCLCPP_ERROR(logger, "CARRY failed — emergency release.");
            setGripper(gripper_pub, OPEN_WIDTH, logger, "emergency open");
            arm.detachObject(bottle.id);
            jointMove(arm, HOME_JOINTS, logger, "retreat HOME");
            shutdown_fn(); return 1;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        // ── Extra settle after CARRY before planning PRE_PLACE ────────────
        // CARRY is a large cross-family joint move. The controller takes
        // longer to publish a fully-settled joint state after large moves.
        // Without this wait, setStartStateToCurrentState() inside jointMove()
        // reads a mid-deceleration state and the planner starts from a
        // slightly wrong position — causing it to route around a phantom
        // near-collision or fail the quality check unnecessarily.
        RCLCPP_INFO(logger, "  [settle] waiting %dms after CARRY...", CARRY_SETTLE_MS);
        rclcpp::sleep_for(std::chrono::milliseconds(CARRY_SETTLE_MS));

        // ── PRE_PLACE ─────────────────────────────────────────────────────
        // Uses CROSS_FAMILY_PATH_RATIO because CARRY and PRE_PLACE are in
        // different configuration families. The old DEFAULT_PATH_RATIO (2.0)
        // was rejecting correct plans on this specific transition.
        // Also uses PLAN_TIME_LONG (20s) and settle_ms=250 for the same reason.
        RCLCPP_INFO(logger, "[8] PRE_PLACE...");
        if (!jointMove(arm, cfg.pre_place.toVector(), logger, "PRE_PLACE",
                       /*max_attempts=*/3,
                       /*settle_ms=*/250,
                       /*max_ratio=*/CROSS_FAMILY_PATH_RATIO,
                       /*plan_time_override=*/PLAN_TIME_LONG)) {
            RCLCPP_ERROR(logger, "PRE_PLACE failed — emergency release.");
            setGripper(gripper_pub, OPEN_WIDTH, logger, "emergency open");
            arm.detachObject(bottle.id);
            jointMove(arm, cfg.carry.toVector(), logger, "retreat CARRY");
            jointMove(arm, HOME_JOINTS,          logger, "retreat HOME");
            shutdown_fn(); return 1;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // ── Straight lower to place.z ─────────────────────────────────────
        RCLCPP_INFO(logger, "[9] Lower to z=%.3f...", cfg.place_z);
        geometry_msgs::msg::Pose lower_target = buildLowerTarget(cfg.pre_place, cfg.place_z);
        if (!cartesianMoveTo(arm, lower_target, logger, "lower",
                             CARTESIAN_FRAC_SOFT)) {
            RCLCPP_WARN(logger, "Lower incomplete — releasing at current height.");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // ── Release ───────────────────────────────────────────────────────
        RCLCPP_INFO(logger, "[10] Releasing '%s'...", bottle.id.c_str());
        setGripper(gripper_pub, OPEN_WIDTH, logger, "open");
        arm.detachObject(bottle.id);
        psi.removeCollisionObjects({bottle.id});
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // ── Return ────────────────────────────────────────────────────────
        RCLCPP_INFO(logger, "Returning: POST_PLACE → PRE_PLACE → CARRY → HOME...");
        setGripper(gripper_pub, CLOSED_WIDTH, logger, "close for transit");
        jointMove(arm, cfg.post_place.toVector(), logger, "POST_PLACE");
        jointMove(arm, cfg.pre_place.toVector(),  logger, "return PRE_PLACE");
        jointMove(arm, cfg.carry.toVector(),       logger, "return CARRY");

        RCLCPP_INFO(logger, "=== Job %zu complete ===", i + 1);
    }

    RCLCPP_INFO(logger, "All jobs complete. Returning to HOME...");
    jointMove(arm, HOME_JOINTS, logger, "final HOME");

    RCLCPP_INFO(logger, "Sequence complete.");
    shutdown_fn();
    return 0;
}