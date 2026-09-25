#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // The two things the planner can be doing (like a string state, but typos won't compile)
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    robot::PlannerCore planner_;

    // Functions ROS calls for you
    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void timerCallback();

    // Helpers
    void planAndPublish();        // run A* from the robot to the goal and publish /path
    void publishEmptyPath();      // tell control "nothing to follow"
    void setState(State new_state);
    static const char * stateName(State state);

    // Connections
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Notebook: latest messages, shared between the functions
    State state_ = State::WAITING_FOR_GOAL;
    nav_msgs::msg::OccupancyGrid map_;         // latest /map
    geometry_msgs::msg::PointStamped goal_;    // current goal
    double robot_x_ = 0.0;                     // latest robot position from /odom/filtered
    double robot_y_ = 0.0;
    bool has_map_ = false;
    bool has_odom_ = false;
};

#endif
