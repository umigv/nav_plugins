#ifndef PLANNER_SERVER_PATH_PLANNER_HPP
#define PLANNER_SERVER_PATH_PLANNER_HPP

#include <vector>
#include <functional>

#include "infra_common/costmap.hpp"

namespace planner_server
{

struct Coordinate
{
    int x;
    int y;
};

class PathPlanner
{
public:
    // Plugins should implement this function
    // Returns a vector of coordinates representing a path from start to goal
    // If no path is found, an empty vector should be returned
    // drivable is given the cost value of a costmap cell and returns whether that
    // cell is drivable
    virtual std::vector<Coordinate> FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        Coordinate start,
        Coordinate goal) = 0;
    virtual ~PathPlanner() {}

protected:
    // pluginlib requires default constructor and destructor
    PathPlanner() {}
};

} // namespace planner_server


#endif