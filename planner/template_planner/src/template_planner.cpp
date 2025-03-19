#include "template_planner.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace infra_common;
using namespace planner_plugins;

std::vector<CellCoordinate> TemplatePlanner::find_path(const Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const CellCoordinate &start,
        const CellCoordinate &goal) 
{
    (void)(costmap);
    (void)(drivable);
    (void)(start);
    (void)(goal);
    return {};
}
