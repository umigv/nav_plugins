#include "example_path_planner_plugin/example_path_planner_plugin.hpp"

#include "rclcpp/rclcpp.hpp"

namespace example_path_planner_plugin
{

ExamplePathPlannerPlugin::ExamplePathPlannerPlugin()
{
}

ExamplePathPlannerPlugin::~ExamplePathPlannerPlugin()
{
}

void ExamplePathPlannerPlugin::test()
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "Hello from ExamplePathPlannerPlugin");
}

}  // namespace example_path_planner_plugin
