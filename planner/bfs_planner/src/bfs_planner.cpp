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
    RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "BfsPlanner finding path");  

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
    // RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "HERE 1");  

    while (!queue.empty()) {
        CellCoordinate current = queue.front();
        // RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "HERE 2");  

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
            // RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "HERE 3");  

            if (costmap.InBounds(nx, ny) && !visited[nx][ny]) {
                const int cost = costmap.GetCost(nx, ny);
                // RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "HERE 4");  
                
                if (drivable(cost)) {
                    visited[nx][ny] = true;
                    parent[nx][ny] = current;
                    queue.push({nx, ny});
                }
            } 
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), "HERE 5");  

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
    
    if (!path.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("BfsPlanner"), 
                    "Path found with %zu points. Start: (%d, %d), Goal: (%d, %d)", 
                    path.size(), path.front().x, path.front().y, path.back().x, path.back().y);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("BfsPlanner"), "Path is empty after reconstruction");
    }

    return path;
}
