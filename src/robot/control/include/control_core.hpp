#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// ---------------- Control settings ----------------
// Speeds are in SIM time (the sim runs ~8x slower than real time). Timeouts are in REAL seconds,
// except PROGRESS_WINDOW, which is measured with odom stamps (sim time).
constexpr double LOOKAHEAD = 1.0;          // metres: how far ahead on the path the carrot is
constexpr double LINEAR_SPEED = 0.5;       // m/s forward while TRACKING
constexpr double MAX_ANGULAR = 1.0;        // rad/s: turn-rate cap while TRACKING, and the turn rate while SPINNING
constexpr double GOAL_TOLERANCE = 0.5;     // metres: must equal the planner's robot::GOAL_TOLERANCE
constexpr double SPIN_ENTER = 1.047;       // rad (60 deg): carrot further off than this -> turn on the spot
constexpr double SPIN_EXIT = 0.524;        // rad (30 deg): carrot back within this -> drive again
constexpr double SPIN_LIMIT = 2.0 * M_PI;  // rad: a full turn without facing the carrot = it never will
constexpr double PATH_TIMEOUT = 2.0;       // real s without a new /path -> stop (the planner went quiet)
constexpr double ODOM_TIMEOUT = 1.0;       // real s without an odom whose stamp CHANGED -> stop (sim link dead)
constexpr double SAME_GOAL_EPS = 0.01;     // metres: goals closer than this are the same goal
constexpr int STOP_BURST = 3;              // ticks of zero Twist sent when stopping
constexpr int CONTROL_PERIOD_MS = 100;     // control loop period (10 Hz)
constexpr double PROGRESS_WINDOW = 5.0;    // sim s: TRACKING this long...
constexpr double PROGRESS_DIST = 0.10;     // ...while moving less than this (metres) = blocked by something

// A point seen from the robot: x = metres forward, y = metres to the left
struct RobotFramePoint
{
  double x;
  double y;
};

// The ROS-free math: yaw, distances, carrot search, frames, velocity. The node does topics, timer and states.
class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    // Heading in radians (0 = facing world +x, counter-clockwise positive) from a quaternion
    double extractYaw(const geometry_msgs::msg::Quaternion & q) const;

    // Wrap any angle into [-pi, pi], e.g. 350 deg -> -10 deg
    double normalizeAngle(double angle) const;

    // Straight-line distance between two points (metres)
    double computeDistance(double x1, double y1, double x2, double y2) const;

    // Index of the path pose nearest to (x, y). The path must not be empty.
    std::size_t closestPoseIndex(
      const std::vector<geometry_msgs::msg::PoseStamped> & poses, double x, double y) const;

    // The carrot: start at the nearest pose, walk forward to the first pose at least LOOKAHEAD
    // away from (x, y). If none is that far, the last pose. Returns its index. The path must not be empty.
    std::size_t findLookaheadPoint(
      const std::vector<geometry_msgs::msg::PoseStamped> & poses, double x, double y) const;

    // A world offset (dx, dy) seen from a robot facing `yaw`: rotate it by -yaw
    RobotFramePoint toRobotFrame(double dx, double dy, double yaw) const;

    // TRACKING command for a carrot in the robot frame: constant forward speed, and the turn rate
    // that puts the robot on the circle through the carrot (pure pursuit), capped at MAX_ANGULAR
    geometry_msgs::msg::Twist computeVelocity(const RobotFramePoint & carrot) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
