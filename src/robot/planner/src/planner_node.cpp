#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // /map: map_memory publishes it RELIABLE + VOLATILE, which is what the default (depth 10) subscription uses
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  // /goal_point: Foxglove publishes it TRANSIENT_LOCAL (it keeps the last click). We subscribe with the
  // default VOLATILE on purpose, so a restarted planner doesn't pick up an old click and drive off by itself.
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Every 500 ms: goal reached? If not, replan from wherever the robot is now
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner started in state %s", stateName(state_));
}

// New map: remember it. If we're heading to a goal, the map may have new walls, so replan.
// A map update never changes the state.
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  map_ = *msg;
  has_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublish();
  }
}

// New goal (clicked in Foxglove): replaces any old goal, then plan straight away
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  goal_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f) in frame '%s'",
    goal_.point.x, goal_.point.y, goal_.header.frame_id.c_str());

  // The goal's numbers only make sense on the map if both are in the same frame
  if (has_map_ && goal_.header.frame_id != map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal frame '%s' is not the map frame '%s': the path will be wrong",
      goal_.header.frame_id.c_str(), map_.header.frame_id.c_str());
  }

  setState(State::WAITING_FOR_ROBOT_TO_REACH_GOAL);
  planAndPublish();
}

// New odometry: just remember where the robot is (/odom/filtered is in the same frame as /map)
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  has_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;  // no goal: nothing to do
  }

  if (has_odom_) {
    const double distance = std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_);
    if (distance < robot::GOAL_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(), "Goal reached (%.2f m away)", distance);
      publishEmptyPath();  // tells control to stop
      setState(State::WAITING_FOR_GOAL);
      return;
    }
  }

  planAndPublish();  // not there yet: replan from the current position
}

void PlannerNode::planAndPublish() {
  // 1. Need a map and a robot position before we can plan anything
  if (!has_map_ || !has_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Can't plan yet: waiting for %s", !has_map_ ? "/map" : "/odom/filtered");
    return;
  }

  // 2. Robot and goal: metres -> grid cells
  const auto start = planner_.worldToGrid(map_, robot_x_, robot_y_);
  const auto goal = planner_.worldToGrid(map_, goal_.point.x, goal_.point.y);
  if (!start) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Robot (%.2f, %.2f) is outside the map: not planning", robot_x_, robot_y_);
    return;
  }
  if (!goal) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Goal (%.2f, %.2f) is outside the map: not planning", goal_.point.x, goal_.point.y);
    return;
  }

  // 3. A goal inside a wall (or too close to one for the robot to fit) can never be reached
  if (planner_.isLethal(map_, *goal)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Goal (%.2f, %.2f) is inside an obstacle (or within 1.3 m of one): not planning",
      goal_.point.x, goal_.point.y);
    return;
  }

  // 4. A*
  const std::vector<robot::CellIndex> cells = planner_.planPath(map_, *start, *goal);
  if (cells.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "No path found to goal (%.2f, %.2f)", goal_.point.x, goal_.point.y);
    return;  // keep the old path and the current state
  }

  // 5. Cells -> Path message (one pose per cell centre, start -> goal)
  nav_msgs::msg::Path path;
  path.header.frame_id = map_.header.frame_id;  // same frame as the map the cells came from
  path.header.stamp = this->now();

  for (const auto & cell : cells) {
    const auto [x, y] = planner_.gridToWorld(map_, cell);  // like Python: x, y = grid_to_world(...)
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;  // no rotation
    path.poses.push_back(pose);
  }

  // The last cell's centre can be up to 7 cm off the clicked point, so end exactly on the goal
  path.poses.back().pose.position.x = goal_.point.x;
  path.poses.back().pose.position.y = goal_.point.y;

  path_pub_->publish(path);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Path published: %zu poses", path.poses.size());
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  // Same frame as normal paths. (No map yet is only possible if we never planned, so use the goal's frame.)
  path.header.frame_id = has_map_ ? map_.header.frame_id : goal_.header.frame_id;
  path.header.stamp = this->now();
  path_pub_->publish(path);  // zero poses
}

void PlannerNode::setState(State new_state) {
  if (new_state == state_) {
    return;  // only log real changes
  }
  RCLCPP_INFO(this->get_logger(), "State: %s -> %s", stateName(state_), stateName(new_state));
  state_ = new_state;
}

const char * PlannerNode::stateName(State state) {
  switch (state) {
    case State::WAITING_FOR_GOAL:
      return "WAITING_FOR_GOAL";
    case State::WAITING_FOR_ROBOT_TO_REACH_GOAL:
      return "WAITING_FOR_ROBOT_TO_REACH_GOAL";
  }
  return "UNKNOWN";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
