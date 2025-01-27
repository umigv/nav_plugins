#include "example_path_planner_plugin/example_path_planner_plugin.hpp"

#include "rclcpp/rclcpp.hpp"

ExamplePathPlannerPlugin::ExamplePathPlannerPlugin()
{
}

ExamplePathPlannerPlugin::~ExamplePathPlannerPlugin()
{
}

std::vector<infra_common::Coordinate> ExamplePathPlannerPlugin::FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        infra_common::Coordinate start,
        infra_common::Coordinate goal)
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "ExamplePathPlannerPlugin finding path");
    return {{0, 0}};
}