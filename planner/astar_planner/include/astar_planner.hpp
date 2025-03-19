#pragma once

#include "plugin_base_classes/path_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <algorithm>

namespace planner_plugins
{

    
class AstarPlanner : public plugin_base_classes::PathPlanner
{
public:
    std::vector<infra_common::CellCoordinate> find_path(const infra_common::Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const infra_common::CellCoordinate &start,
        const infra_common::CellCoordinate &goal) override;
private:


struct Point {
    std::pair<int, int> pos;
    double g_cost;
    double h_cost;
    bool visited; // is in closed set
    bool queued; // is in open list
    Point *parent;

    Point(const std::pair<int, int> pos_) :
        pos(pos_), g_cost(1000), visited(false), queued(false),parent(nullptr) {};

    double f_cost() const {
        return g_cost + h_cost;
    }
};

struct ComparePointsCost {
    bool operator()(const Point* p1, const Point* p2) const {
        return p1->f_cost() > p2->f_cost(); // Min-heap based on f_cost
    }
};

std::vector<std::vector<Point>> grid;
std::priority_queue<Point*, std::vector<Point*>, ComparePointsCost> open;
std::pair<double, double> goal;
std::pair<double, double> start;

std::vector<Point*> find_neighbors(Point* p, const infra_common::Costmap &costmap, const std::function<bool(int)> &drivable);
double h_cost_calculation(Point* current_point, const std::pair<int, int>& goal);
double g_cost_calculation(Point* current_point, Point* parent);
void grid_init(const infra_common::Costmap &costmap);
Point* astar_alg(const std::pair<int, int>& start, const std::pair<int, int>& goal, const infra_common::Costmap &costmap, const std::function<bool(int)> &drivable);
std::vector<infra_common::CellCoordinate> recontruct_path(Point* current);
};

}  // namespace planner_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(planner_plugins::AstarPlanner, plugin_base_classes::PathPlanner)
