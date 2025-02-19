#include "planner_plugins/example_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <queue>
#include <algorithm>

using namespace infra_common;
using namespace planner_plugins;

std::vector<CellCoordinate> ExamplePlanner::find_path(const Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const CellCoordinate &start,
        const CellCoordinate &goal) 
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePlanner"), "ExamplePlanner finding path using BFS");  

    const int width = costmap.GetWidth();
    const int height = costmap.GetHeight();

    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
    std::vector<std::vector<CellCoordinate>> parent(width, std::vector<CellCoordinate>(height, {-1, -1}));
    std::queue<CellCoordinate> queue;

    // Movement directions (right, left, up, down)
    const std::vector<CellCoordinate> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    // Initialize BFS
    queue.push(start);
    visited[start.x][start.y] = true;
    parent[start.x][start.y] = {-1, -1};

    bool found = false;

    while (!queue.empty()) {
        CellCoordinate current = queue.front();
        queue.pop();

        // Early exit if goal found
        if (current.x == goal.x && current.y == goal.y) {
            found = true;
            break;
        }

        // Explore neighbors
        for (const auto& dir : directions) {
            const int nx = current.x + dir.x;
            const int ny = current.y + dir.y;

            // Check if neighbor is valid and unvisited
            if (costmap.InBounds(nx, ny) && !visited[nx][ny]) {
                const int cost = costmap.GetCost(nx, ny);
                if (drivable(cost)) {
                    visited[nx][ny] = true;
                    parent[nx][ny] = current;
                    queue.push({nx, ny});
                }
            }
        }
    }

    if (!found) {
        return {};
    }

    // Reconstruct path
    std::vector<CellCoordinate> path;
    CellCoordinate current = goal;
    while (!(current.x == -1 && current.y == -1)) {
        path.push_back(current);
        current = parent[current.x][current.y];
    }

    std::reverse(path.begin(), path.end());

    return path;
}
