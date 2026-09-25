#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

std::optional<CellIndex> PlannerCore::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid, double x, double y) const
{
  const double res = grid.info.resolution;
  // floor first (so -0.3 -> -1, not 0), then the cast is safe: the value is already a whole number
  const int col = static_cast<int>(std::floor((x - grid.info.origin.position.x) / res));
  const int row = static_cast<int>(std::floor((y - grid.info.origin.position.y) / res));

  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (col < 0 || col >= width || row < 0 || row >= height) {
    return std::nullopt;
  }
  return CellIndex(col, row);
}

std::pair<double, double> PlannerCore::gridToWorld(
  const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const
{
  const double res = grid.info.resolution;
  // + 0.5 = the middle of the cell, not its corner
  const double x = grid.info.origin.position.x + (cell.x + 0.5) * res;
  const double y = grid.info.origin.position.y + (cell.y + 0.5) * res;
  return {x, y};
}

int PlannerCore::cellValue(const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const
{
  const int value = grid.data[cell.y * static_cast<int>(grid.info.width) + cell.x];  // row * width + col
  return value < 0 ? 0 : value;  // unknown (-1) counts as free
}

bool PlannerCore::isLethal(const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & cell) const
{
  return cellValue(grid, cell) >= LETHAL_COST;
}

std::vector<CellIndex> PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid & grid, const CellIndex & start, const CellIndex & goal) const
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);

  // h: straight-line distance to the goal, in cells. Never more than the real cost, so A* stays correct.
  auto heuristic = [&goal](const CellIndex & cell) {
    return std::hypot(static_cast<double>(cell.x - goal.x), static_cast<double>(cell.y - goal.y));
  };

  // open:      to-do list, cheapest f on top          (Python: heapq list)
  // g_score:   cheapest known cost from start to cell (Python: dict {(x, y): cost})
  // came_from: the cell we reached each cell from     (Python: dict {(x, y): (x, y)})
  // closed:    cells whose cheapest cost is final     (Python: set of (x, y))
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start));

  // The 8 neighbours: 4 straight, then 4 diagonal
  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();

    // A cell can be in the queue several times (each time we found a cheaper way to it).
    // The first copy popped is the cheapest; any later copy is stale, so skip it.
    if (closed.count(current) > 0) {
      continue;
    }
    closed.insert(current);

    if (current == goal) {
      // Follow came_from backwards goal -> start, then flip it to start -> goal
      std::vector<CellIndex> path;
      CellIndex cell = goal;
      path.push_back(cell);
      while (cell != start) {
        cell = came_from.at(cell);
        path.push_back(cell);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const int current_value = cellValue(grid, current);
    const double current_g = g_score.at(current);

    for (int i = 0; i < 8; ++i) {
      const CellIndex next(current.x + dxs[i], current.y + dys[i]);
      if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height) {
        continue;  // off the map
      }
      if (closed.count(next) > 0) {
        continue;  // already final
      }

      const int next_value = cellValue(grid, next);
      if (next_value >= LETHAL_COST) {
        // Normally a lethal cell is a wall. One exception, "downhill escape": if we're ALREADY in the
        // lethal zone (only possible at the start, e.g. after teleoping near a wall), we may step to a
        // cell with a strictly lower value. That always leads away from obstacles, never into one (100).
        const bool downhill_escape = current_value >= LETHAL_COST && next_value < current_value;
        if (!downhill_escape) {
          continue;
        }
      }

      // Cost of the step: its length (1 straight, sqrt(2) diagonal) + a penalty for being near obstacles
      const bool diagonal = dxs[i] != 0 && dys[i] != 0;
      const double step_length = diagonal ? std::sqrt(2.0) : 1.0;
      const double new_g = current_g + step_length + COST_WEIGHT * (next_value / 100.0);

      // Only keep it if it's the cheapest way to `next` found so far
      const auto it = g_score.find(next);
      if (it == g_score.end() || new_g < it->second) {
        g_score[next] = new_g;
        came_from[next] = current;
        open.emplace(next, new_g + heuristic(next));
      }
    }
  }

  return {};  // to-do list ran out before reaching the goal: no path
}

}
