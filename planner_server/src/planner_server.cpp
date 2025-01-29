#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "infra_interfaces/action/navigate_to_goal.hpp"
#include "infra_interfaces/msg/coordinate2_d.hpp"
#include "planner_server/path_planner.hpp"

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
            std::shared_ptr<PathPlanner> planner = planner_loader.createSharedInstance("ExamplePathPlannerPlugin");
            // Create dummy costmap for now
            auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            costmap->data = {0, 0, 0, 0};
            costmap->info.width = 2;
            costmap->info.height = 2;

            auto drivable = [](int cost) { return cost == 0; };
            infra_interfaces::msg::Coordinate2D start, goal;
            start.x = 0;
            start.y = 0;
            goal.x = 1;
            goal.y = 1;
            auto path = planner->FindPath(infra_common::Costmap(costmap), 
                drivable,
                start,
                goal);
            RCLCPP_INFO(this->get_logger(), "Found path with length %ld", path.size());

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