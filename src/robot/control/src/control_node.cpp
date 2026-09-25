#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "control_node.hpp"

namespace
{
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// printf-style text into a std::string (Python: an f-string)
template<typename ... Args>
std::string format(const char * fmt, Args... args)
{
  char buffer[200];
  std::snprintf(buffer, sizeof(buffer), fmt, args...);
  return std::string(buffer);
}
}  // namespace

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Every 100 ms: decide what to do and, unless idle, send a command
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(robot::CONTROL_PERIOD_MS), std::bind(&ControlNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Control started in state %s", stateName(state_));
}

// New odometry: remember where the reference point (the lidar) is and which way the robot faces
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = control_.extractYaw(msg->pose.pose.orientation);
  odom_frame_ = msg->header.frame_id;

  // odometry_spoof re-sends the last known pose 10x a second even if the sim link dies,
  // so only an odom with a NEW stamp counts as fresh. (Stamps are compared with each other, never with now().)
  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  if (!has_odom_ || stamp != odom_stamp_) {
    odom_stamp_ = stamp;
    odom_time_ = steadyNow();
  }
  has_odom_ = true;
}

// New path from the planner (~3 per second while it has a goal): keep it for the timer
void ControlNode::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg) {
  // The path's numbers only mean something if they're in the same frame as the odom
  if (has_odom_ && msg->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock_, 2000,
      "Ignoring /path in frame '%s': odom is in '%s'", msg->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }

  path_ = *msg;  // while following, a newer path simply replaces the old one
  path_time_ = steadyNow();
  has_path_ = true;
  new_path_ = true;

  // A path to a different goal means a new click: forget the goal we gave up on
  if (has_blocked_goal_ && !path_.poses.empty()) {
    const auto & goal = path_.poses.back().pose.position;
    if (control_.computeDistance(goal.x, goal.y, blocked_goal_x_, blocked_goal_y_) > robot::SAME_GOAL_EPS) {
      has_blocked_goal_ = false;
      RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f): forgetting the blocked goal", goal.x, goal.y);
    }
  }
}

void ControlNode::timerCallback() {
  if (state_ == State::IDLE && !tryStartFollowing()) {
    // Standing still: finish the stop burst, then send NOTHING, so Foxglove teleop can drive the robot
    if (stop_ticks_left_ > 0) {
      publishCommand(0.0, 0.0);
      --stop_ticks_left_;
    }
    return;
  }
  followPath();
}

// IDLE: start only when a NEW path arrived and it makes sense to follow it
bool ControlNode::tryStartFollowing() {
  if (!new_path_) {
    return false;
  }
  new_path_ = false;  // each path gets one chance to start us

  if (path_.poses.empty()) {
    return false;  // the planner's "goal reached" message
  }
  if (!has_odom_ || odomAge() > robot::ODOM_TIMEOUT) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock_, 2000, "Got a path but odom is stale: not starting");
    return false;
  }
  if (path_.header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock_, 2000,
      "Path frame '%s' is not odom frame '%s': not starting", path_.header.frame_id.c_str(), odom_frame_.c_str());
    return false;
  }

  const auto & goal = path_.poses.back().pose.position;
  if (control_.computeDistance(robot_x_, robot_y_, goal.x, goal.y) <= robot::GOAL_TOLERANCE) {
    return false;  // already there (e.g. one last path sent just before the planner noticed)
  }
  if (has_blocked_goal_ &&
    control_.computeDistance(goal.x, goal.y, blocked_goal_x_, blocked_goal_y_) <= robot::SAME_GOAL_EPS)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock_, 10000,
      "Ignoring paths to blocked goal (%.2f, %.2f): click a different goal", goal.x, goal.y);
    return false;
  }

  // Go: turn on the spot first if the carrot is far to the side or behind, otherwise just drive
  const robot::RobotFramePoint carrot = carrotInRobotFrame();
  const double alpha = std::atan2(carrot.y, carrot.x);
  const std::string reason =
    format("new path to (%.2f, %.2f), carrot %.0f deg off", goal.x, goal.y, alpha * RAD_TO_DEG);
  if (std::abs(alpha) > robot::SPIN_ENTER) {
    startSpinning(alpha, reason);
  } else {
    startTracking(reason);
  }
  return true;
}

// One control step while following: reasons to stop, then the carrot, the mode, the guards and the command
void ControlNode::followPath() {
  // 1. Reasons to stop, checked in this order
  if (odomAge() > robot::ODOM_TIMEOUT) {
    stop("odom stale", false);
    return;
  }
  if (pathAge() > robot::PATH_TIMEOUT) {
    stop("path stale", false);
    return;
  }
  if (path_.poses.empty()) {
    stop("path empty", false);
    return;
  }
  const auto & goal = path_.poses.back().pose.position;
  const double goal_distance = control_.computeDistance(robot_x_, robot_y_, goal.x, goal.y);
  if (goal_distance < robot::GOAL_TOLERANCE) {
    stop(format("goal reached (%.2f m away)", goal_distance), false);
    return;
  }

  // 2. The carrot seen from the robot. alpha = how far off to the side it is (0 = dead ahead, +left)
  const robot::RobotFramePoint carrot = carrotInRobotFrame();
  const double alpha = std::atan2(carrot.y, carrot.x);

  // 3. Switch mode. Entering and leaving use different angles, so it can't flicker between them.
  if (state_ == State::TRACKING && std::abs(alpha) > robot::SPIN_ENTER) {
    startSpinning(alpha, format("carrot %.0f deg off (over %.0f)", alpha * RAD_TO_DEG, robot::SPIN_ENTER * RAD_TO_DEG));
  } else if (state_ == State::SPINNING && std::abs(alpha) < robot::SPIN_EXIT) {
    startTracking(format("carrot %.0f deg off (under %.0f)", alpha * RAD_TO_DEG, robot::SPIN_EXIT * RAD_TO_DEG));
  }

  // 4. Guards, then the command
  double linear = 0.0;
  double angular = 0.0;
  if (state_ == State::SPINNING) {
    // Add up how far we've turned. Each step is wrapped, so crossing +-180 deg isn't counted as a full turn.
    spin_turned_ += control_.normalizeAngle(robot_yaw_ - last_yaw_);
    last_yaw_ = robot_yaw_;
    if (std::abs(spin_turned_) > robot::SPIN_LIMIT) {
      // The lidar point is 1.3 m ahead of the wheels, so a goal close to the wheels can never come in front
      stop(format("stuck spinning (turned %.0f deg, carrot still %.0f deg off)",
        std::abs(spin_turned_) * RAD_TO_DEG, alpha * RAD_TO_DEG), true);
      return;
    }
    angular = spin_direction_ * robot::MAX_ANGULAR;
  } else {
    // No-progress guard: while driving, the robot must move PROGRESS_DIST every PROGRESS_WINDOW (sim time)
    const double moved = control_.computeDistance(anchor_x_, anchor_y_, robot_x_, robot_y_);
    if (moved >= robot::PROGRESS_DIST) {
      anchor_x_ = robot_x_;
      anchor_y_ = robot_y_;
      anchor_stamp_ = odom_stamp_;
    } else if (odom_stamp_ - anchor_stamp_ >= robot::PROGRESS_WINDOW) {
      stop(format("no progress (moved %.2f m in %.1f sim s)", moved, odom_stamp_ - anchor_stamp_), true);
      return;
    }
    const geometry_msgs::msg::Twist cmd = control_.computeVelocity(carrot);
    linear = cmd.linear.x;
    angular = cmd.angular.z;
  }
  publishCommand(linear, angular);

  // At most one status line per real second
  RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock_, 1000,
    "%s | carrot x_r=%.2f y_r=%.2f | alpha %.0f deg | goal %.2f m | cmd v=%.2f w=%.2f",
    stateName(state_), carrot.x, carrot.y, alpha * RAD_TO_DEG, goal_distance, linear, angular);
}

robot::RobotFramePoint ControlNode::carrotInRobotFrame() {
  const std::size_t i = control_.findLookaheadPoint(path_.poses, robot_x_, robot_y_);
  const auto & p = path_.poses[i].pose.position;
  return control_.toRobotFrame(p.x - robot_x_, p.y - robot_y_, robot_yaw_);
}

// Turn on the spot toward the carrot. The direction is picked once and kept for the whole spin,
// so a carrot almost straight behind can't flip it back and forth.
void ControlNode::startSpinning(double alpha, const std::string & reason) {
  spin_direction_ = alpha >= 0.0 ? 1.0 : -1.0;  // carrot on the left -> turn left (positive angular.z)
  spin_turned_ = 0.0;
  last_yaw_ = robot_yaw_;
  setState(State::SPINNING, reason);
}

void ControlNode::startTracking(const std::string & reason) {
  anchor_x_ = robot_x_;  // restart the no-progress clock from here
  anchor_y_ = robot_y_;
  anchor_stamp_ = odom_stamp_;
  setState(State::TRACKING, reason);
}

// Back to IDLE. block_goal = give up on this goal until a different one is clicked.
void ControlNode::stop(const std::string & reason, bool block_goal) {
  if (block_goal && !path_.poses.empty()) {
    has_blocked_goal_ = true;
    blocked_goal_x_ = path_.poses.back().pose.position.x;
    blocked_goal_y_ = path_.poses.back().pose.position.y;
  }
  setState(State::IDLE, reason);
  new_path_ = false;  // only a path that arrives from now on may start us again

  // The sim keeps doing the last command forever, so say "stop" now and on the next ticks too
  publishCommand(0.0, 0.0);
  stop_ticks_left_ = robot::STOP_BURST - 1;
}

void ControlNode::setState(State new_state, const std::string & reason) {
  if (new_state == state_) {
    return;  // only log real changes
  }
  RCLCPP_INFO(this->get_logger(), "State: %s -> %s (%s)", stateName(state_), stateName(new_state), reason.c_str());
  state_ = new_state;
}

void ControlNode::publishCommand(double linear, double angular) {
  geometry_msgs::msg::Twist cmd;  // all six numbers start at 0
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

double ControlNode::steadyNow() {
  return steady_clock_.now().seconds();
}

double ControlNode::odomAge() {
  return has_odom_ ? steadyNow() - odom_time_ : std::numeric_limits<double>::infinity();
}

double ControlNode::pathAge() {
  return has_path_ ? steadyNow() - path_time_ : std::numeric_limits<double>::infinity();
}

const char * ControlNode::stateName(State state) {
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::SPINNING:
      return "SPINNING";
    case State::TRACKING:
      return "TRACKING";
  }
  return "UNKNOWN";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
