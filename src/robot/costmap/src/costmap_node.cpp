#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "costmap_node.hpp"

// ---------------- Costmap settings ----------------
constexpr double RESOLUTION = 0.1;        // meters per cell
constexpr int GRID_CELLS = 300;           // cells per side: 300 x 0.1 m = 30 m
constexpr double ORIGIN = -15.0;          // position (m) of cell (0,0)'s corner, relative to the lidar:
                                          //   15 m behind and 15 m to the right
constexpr double INFLATION_RADIUS = 1.0;  // meters
constexpr int MAX_COST = 100;             // cost of a cell with an obstacle in it

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // ---- Step 1: fresh blank grid, every cell 0 ("free") ----
  // grid[row][col]: row = y (robot's left), col = x (robot's forward)
  std::vector<std::vector<int8_t>> grid(GRID_CELLS, std::vector<int8_t>(GRID_CELLS, 0));

  // Remember where the obstacles are, so step 4 can inflate around them
  std::vector<std::pair<int, int>> obstacles;  // (row, col)

  // ---- Step 2: each beam -> one cell ----
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double r = scan->ranges[i];

    // Skip inf/NaN (hit nothing) and junk readings too close or too far
    if (!(r > scan->range_min && r < scan->range_max)) {
      continue;
    }

    // Beam i's direction, then distance + angle -> x/y in meters
    const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    const double x = r * std::cos(angle);
    const double y = r * std::sin(angle);

    // x/y in meters -> which cell: measure from the (0,0) corner, divide by cell size
    const int col = static_cast<int>(std::floor((x - ORIGIN) / RESOLUTION));
    const int row = static_cast<int>(std::floor((y - ORIGIN) / RESOLUTION));

    // Skip points that land outside the grid (e.g. a 17 m reading straight ahead)
    if (col < 0 || col >= GRID_CELLS || row < 0 || row >= GRID_CELLS) {
      continue;
    }

    // ---- Step 3: mark the obstacle ----
    grid[row][col] = MAX_COST;
    obstacles.emplace_back(row, col);
  }

  // ---- Step 4: inflate around each obstacle ----
  // Check every cell in a square around the obstacle; only cells inside the radius get a cost.
  const int radius_cells = static_cast<int>(std::ceil(INFLATION_RADIUS / RESOLUTION));  // 10

  for (const auto & [obs_row, obs_col] : obstacles) {
    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
        const int row = obs_row + dr;
        const int col = obs_col + dc;
        if (col < 0 || col >= GRID_CELLS || row < 0 || row >= GRID_CELLS) {
          continue;
        }

        const double distance = std::sqrt(dr * dr + dc * dc) * RESOLUTION;  // meters
        if (distance > INFLATION_RADIUS) {
          continue;  // outside the circle: leave it alone
        }

        const double cost = MAX_COST * (1.0 - distance / INFLATION_RADIUS);
        if (cost > grid[row][col]) {  // never lower a cell's cost
          grid[row][col] = static_cast<int8_t>(cost);
        }
      }
    }
  }

  // ---- Step 5: pack into an OccupancyGrid and publish ----
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = scan->header;  // same timestamp + frame as the scan, so Foxglove draws it around the lidar

  msg.info.resolution = RESOLUTION;
  msg.info.width = GRID_CELLS;
  msg.info.height = GRID_CELLS;
  msg.info.origin.position.x = ORIGIN;
  msg.info.origin.position.y = ORIGIN;
  msg.info.origin.orientation.w = 1.0;  // no rotation

  // Flatten 2D -> 1D, one row after another: index = row * width + col
  msg.data.resize(GRID_CELLS * GRID_CELLS);
  for (int row = 0; row < GRID_CELLS; ++row) {
    for (int col = 0; col < GRID_CELLS; ++col) {
      msg.data[row * GRID_CELLS + col] = grid[row][col];
    }
  }

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
