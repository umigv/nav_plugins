#pragma once

#include "plugin_base_classes/path_planner.hpp"

namespace planner_plugins
{

class BfsPlanner : public plugin_base_classes::PathPlanner
{
public:
    std::vector<infra_common::CellCoordinate> find_path(const infra_common::Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const infra_common::CellCoordinate &start,
        const infra_common::CellCoordinate &goal) override;
};

}  // namespace planner_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(planner_plugins::BfsPlanner, plugin_base_classes::PathPlanner)
