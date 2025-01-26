#ifndef PLANNER_SERVER_COSTMAP_HPP
#define PLANNER_SERVER_COSTMAP_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace planner_server
{

class Costmap
{
public:
    Costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
    : _occupancy_grid(costmap)
    { }

    int GetCost(int x, int y) const
    {
        return _occupancy_grid->data[y * GetWidth() + x];
    }

    int GetWidth() const
    {
        return _occupancy_grid->info.width;
    }

    int GetHeight() const
    {
        return _occupancy_grid->info.height;
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr _occupancy_grid;
    

};

}

#endif