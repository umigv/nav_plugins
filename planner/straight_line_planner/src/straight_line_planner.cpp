#include "straight_line_planner.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace infra_common;
using namespace planner_plugins;

std::vector<CellCoordinate> StraightLinePlanner::find_path(const Costmap &costmap, 
    const std::function<bool(int)> &drivable,
    const CellCoordinate &start,
    const CellCoordinate &goal) 
{
RCLCPP_INFO(rclcpp::get_logger("StraightLinePlanner"), "StraightLinePlanner finding path");  

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

