#ifndef INFRA_COMMON_COSTMAP_HPP
#define INFRA_COMMON_COSTMAP_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"
namespace infra_common
{

    class Costmap
    {
    public:
        Costmap(const nav_msgs::msg::OccupancyGrid occupancy_grid_in);

        int GetCost(int x, int y) const;

        bool InBounds(int x, int y) const;

        int GetWidth() const;

        int GetHeight() const;

        float GetResolution() const;

        std::pair<float, float> GetOrigin() const;

    private:
        nav_msgs::msg::OccupancyGrid _occupancy_grid;
    };

}

#endif