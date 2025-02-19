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
    RCLCPP_INFO(rclcpp::get_logger("ExamplePlanner"), "ExamplePlanner finding path");  
    
    // Straight line planner: just goes up in a straight line until it reaches the goal
    // or hits an obstacle
    std::vector<CellCoordinate> path;
    CellCoordinate curr = start;
    while (true) 
    {
        if (curr == goal)
        {
            break;
        }
        curr.y += 1;
        if (!costmap.InBounds(curr.x, curr.y) ||
            !drivable(costmap.GetCost(curr.x, curr.y))) 
        {
            break;
        }
        path.push_back(curr);
    }

    return path;
}
