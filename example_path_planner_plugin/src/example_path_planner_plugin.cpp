#include "example_path_planner_plugin/example_path_planner_plugin.hpp"

#include "rclcpp/rclcpp.hpp"

ExamplePathPlannerPlugin::ExamplePathPlannerPlugin()
{
}

ExamplePathPlannerPlugin::~ExamplePathPlannerPlugin()
{
}

std::vector<planner_server::Coordinate> ExamplePathPlannerPlugin::FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        planner_server::Coordinate start,
        planner_server::Coordinate goal)
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "ExamplePathPlannerPlugin finding path");
    return {{0, 0}};
}