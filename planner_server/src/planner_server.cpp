#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "../include/planner_server/path_planner.hpp" // don't know why planner_server/path_planner.hpp can't be found


namespace planner_server
{

class PlannerServer : public rclcpp::Node
{
public:
    PlannerServer() 
    : Node("planner_server")
    {
        pluginlib::ClassLoader<PathPlanner> planner_loader("planner_server", "planner_server::PathPlanner");
        try
        {
            std::shared_ptr<PathPlanner> planner = planner_loader.createSharedInstance("example_path_planner_plugin::ExamplePathPlannerPlugin");
            planner->test();
        }
        catch(pluginlib::PluginlibException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "The plugin failed to load. Error: %s", ex.what());
        }
    }
};

} // namespace planner_server

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting planner_server...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planner_server::PlannerServer>());
    rclcpp::shutdown();
    return 0;
}