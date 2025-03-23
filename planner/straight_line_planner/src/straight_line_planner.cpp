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

    // Straight line planner: just goes in the positive x direction until it reaches the goal
    // or hits an obstacle
    std::vector<CellCoordinate> path;
    path.push_back(start);
    CellCoordinate curr = start;
    while (true) 
    {
        if (curr == goal)
        {
            break;
        }
        curr.x += 1;
        RCLCPP_INFO(rclcpp::get_logger("StraightLinePlanner"), "Cost at cell: %d, %d is %d", curr.x, curr.y, costmap.GetCost(curr.x, curr.y));
        if (!costmap.InBounds(curr.x, curr.y) ||
            !drivable(costmap.GetCost(curr.x, curr.y))) 
        {
            RCLCPP_INFO(rclcpp::get_logger("StraightLinePlanner"), "Hit obstacle or went out-of-bounds at cell: %d, %d", curr.x, curr.y);    
            break;
        }
        path.push_back(curr);
        RCLCPP_INFO(rclcpp::get_logger("StraightLinePlanner"), "Adding cell to path: %d, %d", curr.x, curr.y);  
    }

    return path;
}

