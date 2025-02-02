#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "infra_interfaces/action/navigate_to_goal.hpp"
#include "infra_interfaces/msg/coordinate2_d.hpp"
#include "planner_server/path_planner.hpp"

/*
Call NavigateToGoal action with following command:
ros2 action send_goal /navigate_to_goal infra_interfaces/action/NavigateToGoal "{costmap: {header: {frame_id: 'map'}, info: {width: 2, height: 2, resolution: 1.0, origin: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, data: [0, 0, 0, 0]}, start: {x: 0, y: 0}, goal: {x: 1, y: 1}}"
*/

namespace planner_server
{

using namespace infra_common;
using namespace infra_interfaces::msg;

using NavigateToGoal = infra_interfaces::action::NavigateToGoal;
using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

class PlannerServer : public rclcpp::Node
{
public:
    PlannerServer() 
    : Node("planner_server")
    {
        load_planner_plugin();

        using namespace std::placeholders;
        _navigate_server = rclcpp_action::create_server<NavigateToGoal>(this,
            "navigate_to_goal",
            std::bind(&PlannerServer::handle_goal, this, _1, _2),
            std::bind(&PlannerServer::handle_cancel, this, _1),
            std::bind(&PlannerServer::handle_accepted, this, _1));
    }

private:

    void load_planner_plugin()
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
            RCLCPP_ERROR(this->get_logger(), "The planner plugin failed to load. Error: %s", ex.what());
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread(std::bind(&PlannerServer::navigate, this, std::placeholders::_1), goal_handle).detach();
    }

    void navigate(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        auto action_goal = goal_handle->get_goal();
        auto occ_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>(action_goal->costmap);
        Costmap costmap(occ_grid);
        Coordinate2D start = action_goal->start;
        Coordinate2D goal = action_goal->goal;
        RCLCPP_INFO(get_logger(), "Navigating from (%ld, %ld) to (%ld, %ld)", start.x, start.y, goal.x, goal.y);

        auto result = std::make_shared<NavigateToGoal::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Navigation succeeded");
    }

    std::shared_ptr<PathPlanner> _planner;
    rclcpp_action::Server<NavigateToGoal>::SharedPtr _navigate_server;

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