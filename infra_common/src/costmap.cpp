#include "infra_common/costmap.hpp"

using namespace infra_common;

int Costmap::GetCost(int x, int y) const
{
    return _occupancy_grid->data[y * GetWidth() + x];
}

int Costmap::GetWidth() const
{
    return _occupancy_grid->info.width;
}

int Costmap::GetHeight() const
{
    return _occupancy_grid->info.height;
}