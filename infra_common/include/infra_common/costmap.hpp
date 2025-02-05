#ifndef INFRA_COMMON_COSTMAP_HPP
#define INFRA_COMMON_COSTMAP_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace infra_common
{

class Costmap
{
public:
    Costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
    : _occupancy_grid(costmap)
    { }

    int GetCost(int x, int y) const;

    bool InBounds(int x, int y) const;

    int GetWidth() const;

    int GetHeight() const;

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr _occupancy_grid;
};

}

#endif