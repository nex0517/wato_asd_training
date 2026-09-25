#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // IDLE = standing still and silent (teleop works); SPINNING = turning on the spot; TRACKING = driving;
    // REVERSING = backing up to get unstuck, then trying again
    enum class State { IDLE, SPINNING, TRACKING, REVERSING };

    robot::ControlCore control_;

    // Functions ROS calls for you
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void timerCallback();

    // Helpers
    bool tryStartFollowing();                            // IDLE: start if a new, usable path arrived
    void followPath();                                   // SPINNING/TRACKING/REVERSING: one 100 ms step
    robot::RobotFramePoint carrotInRobotFrame();         // lookahead point, seen from the robot
    void startSpinning(double alpha, const std::string & reason);
    void startTracking(const std::string & reason);
    void startDriving(const std::string & why);          // SPINNING or TRACKING, picked from where the carrot is
    void startRecovery(const std::string & reason);      // stuck: back up, or give up after MAX_RECOVERIES
    void stop(const std::string & reason, bool block_goal);  // -> IDLE and start the stop burst
    void setState(State new_state, const std::string & reason);
    void publishCommand(double linear, double angular);
    double steadyNow();                                  // real seconds from a clock that never jumps
    double odomAge();                                    // real s since odom last had a NEW stamp
    double pathAge();                                    // real s since the last /path
    static const char * stateName(State state);

    // Connections
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};       // real time, for ages and log throttling

    // Notebook: latest messages and bookkeeping, shared between the functions
    State state_ = State::IDLE;
    int stop_ticks_left_ = robot::STOP_BURST;             // zeros still to send (starts with the startup burst)

    nav_msgs::msg::Path path_;                           // latest accepted /path
    bool has_path_ = false;
    bool new_path_ = false;                              // a path arrived since IDLE last looked
    double path_time_ = 0.0;                             // steadyNow() when it arrived

    double robot_x_ = 0.0;                               // latest odom: the lidar point (reference point)
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    std::string odom_frame_;
    bool has_odom_ = false;
    double odom_stamp_ = 0.0;                            // odom header stamp (sim s), to spot a frozen sim
    double odom_time_ = 0.0;                             // steadyNow() when that stamp last changed

    double spin_direction_ = 1.0;                        // +1 = turn left, -1 = turn right (fixed per spin)
    double spin_turned_ = 0.0;                           // radians turned during this spin
    double last_yaw_ = 0.0;                              // yaw at the previous tick, to measure the turn

    double anchor_x_ = 0.0;                              // no-progress guard: where TRACKING/REVERSING last
    double anchor_y_ = 0.0;                              //   made PROGRESS_DIST of progress...
    double anchor_stamp_ = 0.0;                          //   ...and when (odom stamp, sim s)
    double spin_anchor_turned_ = 0.0;                    // spin progress: spin_turned_ at the last 10 deg mark...
    double spin_anchor_stamp_ = 0.0;                     //   ...and when (odom stamp, sim s)

    double reverse_start_x_ = 0.0;                       // where the current back-up started
    double reverse_start_y_ = 0.0;
    int recoveries_used_ = 0;                            // back-ups spent on the goal below
    double recovery_goal_x_ = 0.0;
    double recovery_goal_y_ = 0.0;

    bool has_blocked_goal_ = false;                      // goal that ended in "stuck spinning"/"no progress"
    double blocked_goal_x_ = 0.0;
    double blocked_goal_y_ = 0.0;
};

#endif
