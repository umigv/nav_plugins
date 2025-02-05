#include "example_path_planner_plugin/example_path_planner_plugin.hpp"

#include "rclcpp/rclcpp.hpp"

using Coordinate2D = infra_interfaces::msg::Coordinate2D;

ExamplePathPlannerPlugin::ExamplePathPlannerPlugin()
{
}

ExamplePathPlannerPlugin::~ExamplePathPlannerPlugin()
{
}

std::vector<Coordinate2D> ExamplePathPlannerPlugin::FindPath(infra_common::Costmap costmap, 
    std::function<bool(int)> drivable,
    Coordinate2D start,
    Coordinate2D goal)
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "ExamplePathPlannerPlugin planning path");
    
    // Straight line planner: just goes up in a straight line until it reaches the goal
    // or hits an obstacle
    std::vector<Coordinate2D> path;
    Coordinate2D curr = start;
    while (true) 
    {
        if ((curr.x == goal.x) && (curr.y == goal.y))
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