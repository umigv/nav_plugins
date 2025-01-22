#include "rclcpp/rclcpp.hpp"

class PlannerServer : public rclcpp::Node
{
public:
    PlannerServer() 
    : Node("planner_server")
    {
    }
};

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting planner_server...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerServer>());
    rclcpp::shutdown();
    return 0;
}