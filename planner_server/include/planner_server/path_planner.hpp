#ifndef PLANNER_SERVER_PATH_PLANNER_HPP
#define PLANNER_SERVER_PATH_PLANNER_HPP

#include <vector>
#include <functional>

#include "infra_common/costmap.hpp"
#include "infra_interfaces/msg/coordinate2_d.hpp"


namespace planner_server
{

class PathPlanner
{
public:
    // Plugins should implement this function
    // Returns a vector of coordinates representing a path from start to goal; path 
    // should include goal but not start
    // If no path is found, an empty vector should be returned
    // drivable is given the cost value of a costmap cell and returns whether that
    // cell is drivable
    virtual std::vector<infra_interfaces::msg::Coordinate2D> FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        infra_interfaces::msg::Coordinate2D start,
        infra_interfaces::msg::Coordinate2D goal) = 0;
    virtual ~PathPlanner() {}

protected:
    // pluginlib requires default constructor and destructor
    PathPlanner() {}
};

}


#endif