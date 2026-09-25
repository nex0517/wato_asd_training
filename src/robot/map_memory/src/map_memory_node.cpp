#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Doorbells: ROS calls these functions whenever a message arrives
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Megaphone: sends OccupancyGrid messages on /map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Alarm clock: calls updateMap every 1000 ms
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::updateMap, this));

  // The global map: one big fixed grid for the whole world.
  // Every cell starts at 0 (free), same as your costmap, so the planner
  // can plan through places the robot hasn't seen yet.
  global_map_.info.resolution = map_resolution_;
  global_map_.info.width = map_width_;
  global_map_.info.height = map_height_;
  global_map_.info.origin.position.x = map_origin_x_;
  global_map_.info.origin.position.y = map_origin_y_;
  global_map_.info.origin.orientation.w = 1.0;  // no rotation
  global_map_.data.assign(map_width_ * map_height_, 0);
}

// Doorbell 1: save the newest costmap, plus where the robot was at that moment
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!has_odom_) {
    return;  // can't place a costmap until we know where the robot is
  }
  latest_costmap_ = *msg;
  // Remember the pose when the "photo" was taken, not when we paste it later.
  // Otherwise walls smear if the robot moves or turns in between.
  costmap_robot_x_ = robot_x_;
  costmap_robot_y_ = robot_y_;
  costmap_robot_yaw_ = robot_yaw_;
  has_costmap_ = true;
}

// Doorbell 2: save where the robot is and which way it faces (many times a second)
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Heading (yaw) from the quaternion: the standard formula for rotation about the vertical axis
  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  odom_frame_ = msg->header.frame_id;
  has_odom_ = true;
}

// Alarm clock: once a second, decide whether to merge, then publish
void MapMemoryNode::updateMap() {
  // 1. Nothing to do until both doorbells have rung at least once
  if (!has_costmap_ || !has_odom_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for costmap and odom...");
    return;
  }

  // 2. The gate: the first merge always goes through.
  //    After that, only when the robot is at least distance_threshold_ from the pin.
  const double distance = std::hypot(robot_x_ - last_merge_x_, robot_y_ - last_merge_y_);
  if (!has_merged_ || distance >= distance_threshold_) {
    if (has_merged_) {
      RCLCPP_INFO(this->get_logger(), "Moved %.2f m since last merge: merging", distance);
    } else {
      RCLCPP_INFO(this->get_logger(), "First merge");
    }
    integrateCostmap();          // 3. paste the costmap into the global map
    last_merge_x_ = robot_x_;    // 4. move the pin
    last_merge_y_ = robot_y_;
    has_merged_ = true;
  }

  // 5. Publish every tick, not just on merges, so the planner and Foxglove
  //    always have a recent map even if they start listening late
  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = odom_frame_;
  map_pub_->publish(global_map_);
}

// The merge: turn and slide the costmap into place on the global map.
// We loop over MAP cells and look up the costmap cell on top of each one.
// Going this direction leaves no holes, even when the costmap is rotated.
void MapMemoryNode::integrateCostmap() {
  const auto & cm = latest_costmap_;
  const int cm_width = static_cast<int>(cm.info.width);
  const int cm_height = static_cast<int>(cm.info.height);
  const double cm_res = cm.info.resolution;
  const double cm_origin_x = cm.info.origin.position.x;
  const double cm_origin_y = cm.info.origin.position.y;

  // Where the costmap's centre (the lidar) was in the world when the costmap was made
  const double cos_yaw = std::cos(costmap_robot_yaw_);
  const double sin_yaw = std::sin(costmap_robot_yaw_);
  const double lidar_x = costmap_robot_x_ + lidar_offset_x_ * cos_yaw - lidar_offset_y_ * sin_yaw;
  const double lidar_y = costmap_robot_y_ + lidar_offset_x_ * sin_yaw + lidar_offset_y_ * cos_yaw;

  for (int map_row = 0; map_row < map_height_; ++map_row) {
    for (int map_col = 0; map_col < map_width_; ++map_col) {
      // World position of this map cell's centre
      const double world_x = map_origin_x_ + (map_col + 0.5) * map_resolution_;
      const double world_y = map_origin_y_ + (map_row + 0.5) * map_resolution_;

      // Undo the slide, then undo the turn: world -> lidar's point of view
      const double dx = world_x - lidar_x;
      const double dy = world_y - lidar_y;
      const double local_x =  cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;

      // Lidar's point of view -> costmap cell (same math as your costmap node: floor, not round)
      const int cm_col = static_cast<int>(std::floor((local_x - cm_origin_x) / cm_res));
      const int cm_row = static_cast<int>(std::floor((local_y - cm_origin_y) / cm_res));
      if (cm_col < 0 || cm_col >= cm_width || cm_row < 0 || cm_row >= cm_height) {
        continue;  // this map cell is outside the costmap's 30 m window
      }

      const int8_t new_value = cm.data[cm_row * cm_width + cm_col];
      if (new_value < 0) {
        continue;  // unknown in the costmap: keep what the map already has
      }

      // Merge rule: keep the higher value, so remembered walls never get erased
      int8_t & map_value = global_map_.data[map_row * map_width_ + map_col];
      if (new_value > map_value) {
        map_value = new_value;
      }
    }
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
