#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;   // from the template, unused

    // Functions ROS calls for you
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    // Helper: pastes the latest costmap into the global map
    void integrateCostmap();

    // Connections
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Notebook: what the functions pass to each other between calls
    nav_msgs::msg::OccupancyGrid global_map_;      // the big map of the whole world
    nav_msgs::msg::OccupancyGrid latest_costmap_;  // newest costmap (set by costmapCallback)
    double costmap_robot_x_ = 0.0;                 // where the robot was when that costmap arrived
    double costmap_robot_y_ = 0.0;
    double costmap_robot_yaw_ = 0.0;
    double robot_x_ = 0.0;                         // where the robot is now (set by odomCallback)
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;                       // heading in radians (0 = facing world +x)
    double last_merge_x_ = 0.0;                    // the pin: where we last merged
    double last_merge_y_ = 0.0;
    std::string odom_frame_ = "odom";              // frame the odom positions are in (copied from odom)
    bool has_costmap_ = false;
    bool has_odom_ = false;
    bool has_merged_ = false;

    // Settings
    double distance_threshold_ = 1.5;  // metres to move before merging again
    double map_resolution_ = 0.1;      // metres per cell
    int map_width_ = 600;              // cells: 600 x 0.1 m = 60 m
    int map_height_ = 600;
    double map_origin_x_ = -30.0;      // corner of cell (0, 0), so the map is centred on (0, 0)
    double map_origin_y_ = -30.0;
    double lidar_offset_x_ = 0.0;      // lidar position relative to the robot centre (see /tf_static)
    double lidar_offset_y_ = 0.0;
};

#endif
