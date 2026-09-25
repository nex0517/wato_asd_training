#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// ---------------- Planner settings ----------------
// Costmap inflation is 100 * (1 - distance / 1.0 m), so value 30 = 0.7 m from an obstacle.
// 0.7 m is the robot's half-width including wheels, so ">= 30" = "the robot's side would hit something".
// (Not 90: only an obstacle and the 4 cells touching it reach 90, which leaves the robot ~0.1 m of room.)
constexpr int LETHAL_COST = 30;         // cell value >= this: can't enter
constexpr double COST_WEIGHT = 5.0;     // how much a cell's value (0-100) adds to the cost of stepping into it
constexpr double GOAL_TOLERANCE = 0.5;  // metres: robot closer than this to the goal = goal reached

// ---------------- The guide's A* structs ----------------

// A grid cell. x = column, y = row (same as the costmap: col goes with x, row goes with y).
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex & other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex & other) const { return x != other.x || y != other.y; }
};

// Lets CellIndex be a key in std::unordered_map (Python gets this for free with tuple keys).
// Packs x into the top 32 bits and y into the bottom 32 bits, so every cell gets its own number.
// (The guide's x ^ (y << 1) gives hundreds of cells the same hash on a 600x600 map, which makes lookups slow.)
struct CellIndexHash
{
  std::size_t operator()(const CellIndex & idx) const
  {
    const std::uint64_t packed =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(idx.x)) << 32) |
      static_cast<std::uint32_t>(idx.y);
    return std::hash<std::uint64_t>()(packed);
  }
};

// One entry in A*'s to-do list: a cell and its f = g (cost so far) + h (guess of cost left)
struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Tells std::priority_queue to put the SMALLEST f on top (by default it puts the largest on top)
struct CompareF
{
  bool operator()(const AStarNode & a, const AStarNode & b) const
  {
    return a.f_score > b.f_score;
  }
};

// The ROS-free planning logic: grid math + A*. The node does all the topics and timers.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // World position (metres) -> grid cell. std::nullopt (like Python's None) if it's off the grid.
    std::optional<CellIndex> worldToGrid(const nav_msgs::msg::OccupancyGrid & grid, double x, double y) const;

    // Grid cell -> world position (metres) of the cell's centre, as an (x, y) pair
    std::pair<double, double> gridToWorld(const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const;

    // True if the robot isn't allowed to enter this cell
    bool isLethal(const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const;

    // A* from start to goal. Returns the cells in order start -> goal, or an empty vector if there's no path.
    std::vector<CellIndex> planPath(
      const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & start, const CellIndex & goal) const;

  private:
    // A cell's value, with unknown (-1) treated as free (0)
    int cellValue(const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const;

    rclcpp::Logger logger_;
};

}

#endif
