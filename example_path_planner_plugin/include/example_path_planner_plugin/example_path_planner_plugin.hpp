#ifndef EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_
#define EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_

#include "example_path_planner_plugin/visibility_control.h"
#include "planner_server/path_planner.hpp"
#include "planner_server/costmap.hpp"

namespace example_path_planner_plugin
{

class ExamplePathPlannerPlugin : public planner_server::PathPlanner
{
public:
    ExamplePathPlannerPlugin();
    virtual ~ExamplePathPlannerPlugin();

    std::vector<planner_server::Coordinate> FindPath(planner_server::Costmap costmap, 
        std::function<bool(int)> drivable,
        planner_server::Coordinate start,
        planner_server::Coordinate goal) override;
};

}  // namespace example_path_planner_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(example_path_planner_plugin::ExamplePathPlannerPlugin, planner_server::PathPlanner)

#endif  // EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_
