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
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "ExamplePathPlannerPlugin finding path");
    Coordinate2D path_coord;
    path_coord.x = 0;
    path_coord.y = 0;
    return {path_coord};
}