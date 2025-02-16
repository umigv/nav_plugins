#ifndef ASTAR_PLUGIN__ASTAR_PLUGIN_HPP_
#define ASTAR_PLUGIN__ASTAR_PLUGIN_HPP_

#include "astar_plugin/visibility_control.h"
#include "planner_server/path_planner.hpp"
#include "infra_common/costmap.hpp"
#include "astar_plugin/astar_plugin.hpp"

class AstarPlugin : public planner_server::PathPlanner
{
public:
  AstarPlugin();

  virtual ~AstarPlugin();

  std::vector<infra_interfaces::msg::Coordinate2D> FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        infra_interfaces::msg::Coordinate2D start,
        infra_interfaces::msg::Coordinate2D goal) override;

  private:
  class Point {
    std::pair<int, int> pos;
    double g_cost;
    double h_cost;
    int occupied;
    bool visited; // is in closed set
    bool queued; // is in open list
    Point *parent;

    Point(const pair<int, int> pos_, int occupied_) :
        pos(pos_), occupied(occupied_), g_cost(1000), visited(false), queued(false),parent(nullptr) {};

    double f_cost() const {
        return g_cost + h_cost;
    }
  };

  struct ComparePointsCost {
    bool operator()(const Point* p1, const Point* p2) const {
        return p1->f_cost() > p2->f_cost(); // Min-heap based on f_cost
    }
  };

  // Point operator methods
  bool operator<(const Point& lhs, const Point& rhs) { return (lhs.f_cost() < rhs.f_cost()); }
  bool operator>(const Point& lhs, const Point& rhs) { return (rhs < lhs); }
  bool operator<=(const Point& lhs, const Point& rhs) { return !(lhs > rhs); }
  bool operator>=(const Point& lhs, const Point& rhs) { return !(lhs < rhs); }


  std::vector<vector<Point>> grid;
  std::priority_queue<Point*, std::vector<Point*>, ComparePointsCost> open;
  std::pair<double, double> goal;
  std::pair<double, double> start;

  bool is_cell_occupied(int pos1, int pos2);
  bool is_in_grid(int pos1, int pos2);
  std::vector<Point*> find_neighbors(Point* p);
  double h_cost_calculation(Point* current_point, const pair<int, int>& goal);
  double g_cost_calculation(Point* current_point, Point* parent);
  void astar_init(const pair<int, int>& start, const pair<int, int>& goal, std::function<bool(int)> drivable, infra_common::Costmap &costmap)
  Point* astar_alg(const pair<int, int>& start, const pair<int, int>& goal);
  pair<int, int> AstarPlugin::position_to_cell(const Coordinate2D current, const float resolution, const std::pair<float, float> origin);
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(AstarPlugin, planner_server::PathPlanner)

#endif  // ASTAR_PLUGIN__ASTAR_PLUGIN_HPP_
