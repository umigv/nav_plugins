#include "bfs_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <queue>
#include <algorithm>

using namespace infra_common;
using namespace planner_plugins;

std::vector<CellCoordinate> BfsPlanner::find_path(const Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const CellCoordinate &start,
        const CellCoordinate &goal) 
{
    rclcpp::Time now = rclcpp::Clock().now();
    int64_t sec = now.seconds();
    int64_t nanosec = now.nanoseconds();
    int64_t millisec = (nanosec / 1000000) % 1000;
    RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"),
            "BfsPlanner finding path at time: %ld.%03ld", sec, millisec);

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
    now = rclcpp::Clock().now();
    sec = now.seconds();
    nanosec = now.nanoseconds();
    millisec = (nanosec / 1000000) % 1000;
    RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"),
        "BfsPlanner finished reconstructing path at time: %ld.%03ld | Path length: %zu",
        sec, millisec, path.size());
    return path;
}
