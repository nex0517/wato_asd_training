#include <algorithm>
#include <cmath>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger) {}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion & q) const
{
  // Standard formula for the rotation about the vertical axis (same as map_memory)
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double ControlCore::normalizeAngle(double angle) const
{
  // atan2 of (sin, cos) gives back the same direction, always in [-pi, pi]
  return std::atan2(std::sin(angle), std::cos(angle));
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const
{
  return std::hypot(x2 - x1, y2 - y1);
}

std::size_t ControlCore::closestPoseIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses, double x, double y) const
{
  std::size_t best = 0;
  double best_distance = computeDistance(x, y, poses[0].pose.position.x, poses[0].pose.position.y);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double d = computeDistance(x, y, poses[i].pose.position.x, poses[i].pose.position.y);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

std::size_t ControlCore::findLookaheadPoint(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses, double x, double y) const
{
  // Walk forward from where the robot is on the path (never backwards), stop at the first pose far enough away
  for (std::size_t i = closestPoseIndex(poses, x, y); i < poses.size(); ++i) {
    if (computeDistance(x, y, poses[i].pose.position.x, poses[i].pose.position.y) >= LOOKAHEAD) {
      return i;
    }
  }
  return poses.size() - 1;  // the whole rest of the path is close: aim at the goal itself
}

RobotFramePoint ControlCore::toRobotFrame(double dx, double dy, double yaw) const
{
  // Rotating by -yaw turns "east/north" into "forward/left" (Python: the same two lines with math.cos/sin)
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const RobotFramePoint & carrot) const
{
  // Pure pursuit: the circle that starts at the robot, heading forward, and passes through the carrot
  // has curvature 2*y / d^2 (= 2*sin(alpha) / d). Turn rate = speed * curvature.
  const double d = std::hypot(carrot.x, carrot.y);
  const double curvature = d > 1e-6 ? 2.0 * carrot.y / (d * d) : 0.0;

  geometry_msgs::msg::Twist cmd;  // every field starts at 0
  cmd.linear.x = LINEAR_SPEED;
  cmd.angular.z = std::clamp(LINEAR_SPEED * curvature, -MAX_ANGULAR, MAX_ANGULAR);
  return cmd;
}

}
