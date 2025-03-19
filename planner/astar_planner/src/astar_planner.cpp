#include "astar_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <algorithm>

using namespace infra_common;
using namespace planner_plugins;
using namespace std;

vector<AstarPlanner::Point*> AstarPlanner::find_neighbors(AstarPlanner::Point* p, const Costmap &costmap, const std::function<bool(int)> &drivable) {
    vector<AstarPlanner::Point*> neighbors;

    for (int row = -1; row <= 1; row++) {
        for (int col = -1; col <=1; col++) {
            int posx = p->pos.first+row;
            int posy = p->pos.second+col;
            // skip if it is not in grid
            if(posx >= static_cast<int>(grid.size()) 
                || posy >= static_cast<int>(grid[0].size())
                || posx < 0 || posy < 0) 
                continue;
            if(row == 0 && col == 0){
                continue;
            }
            // check if cell is occupied
            const int cost = costmap.GetCost(posx, posy);
            bool drivable_ = drivable(cost);

            if(drivable_ && !grid[posx][posy].visited){
                neighbors.push_back(&grid[posx][posy]);
            }
        }
    }

    return neighbors;
}

double AstarPlanner::h_cost_calculation(AstarPlanner::Point* current_point, const pair<int, int>& goal){
    double intermediate = pow(current_point->pos.first - goal.first, 2) + 
            pow(current_point->pos.second - goal.second, 2);
    return sqrt(intermediate);
}

double AstarPlanner::g_cost_calculation(AstarPlanner::Point* current_point, AstarPlanner::Point* parent){
    double intermediate = pow(current_point->pos.first - parent->pos.first, 2) + 
            pow(current_point->pos.second - parent->pos.second, 2);
    return sqrt(intermediate);
}

void AstarPlanner::grid_init(const Costmap &costmap){
    for(int i = 0; i < costmap.GetWidth(); i++){
        std::vector<Point> gridRow;
        for(int j = 0; j < costmap.GetHeight(); j++){
            std::pair<int, int> pos = {i, j};
            Point newpoint = Point(pos);
            gridRow.push_back(newpoint);
        }
        grid.push_back(gridRow);
    }
}

// returns nullptr if found otherwise returns start_ptr
AstarPlanner::Point* AstarPlanner::astar_alg(const pair<int, int>& start, const pair<int, int>& goal, const Costmap &costmap, const std::function<bool(int)> &drivable){
    //initialization
    AstarPlanner::grid_init(costmap);
    grid[start.first][start.second].g_cost = 0;
    open.push(&grid[start.first][start.second]);
    grid[start.first][start.second].queued = true;
    while(!open.empty()){
        AstarPlanner::Point* current = open.top();
        open.pop();
        current->visited = true;
        if(current->pos == goal) { // not sure
            return current;
        }

        std::vector<AstarPlanner::Point*> neighbors = find_neighbors(current, costmap, drivable);
        for(AstarPlanner::Point* neighbor : neighbors){
            double new_cost = g_cost_calculation(neighbor, current);
            if(neighbor->g_cost > (current->g_cost + new_cost)){
                neighbor->parent = current;
                neighbor->g_cost = current->g_cost + new_cost;
                neighbor->h_cost = h_cost_calculation(neighbor, goal);
            }
            if(!neighbor->queued){
                open.push(neighbor);
                neighbor->queued = true;
            }
        }
    }
    return nullptr;
}   

std::vector<CellCoordinate> AstarPlanner::recontruct_path(AstarPlanner::Point* current){
    if(!current) return {};
    std::vector<CellCoordinate> path;
    while(current){
        CellCoordinate current_cord = {current->pos.first, current->pos.second}; 
        path.push_back(current_cord);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<CellCoordinate> AstarPlanner::find_path(const Costmap &costmap, 
        const std::function<bool(int)> &drivable,
        const CellCoordinate &start,
        const CellCoordinate &goal) 
{
    RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "AstarPlanner finding path");
    std::pair<int, int> start_ = {start.x, start.y};
    std::pair<int, int> goal_ = {goal.x, goal.y}; 
    AstarPlanner::Point* current = astar_alg(start_, goal_, costmap, drivable); 
    RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "AstarPlanner reconstructing path");
    std::vector<CellCoordinate> path = recontruct_path(current); 
    RCLCPP_INFO(rclcpp::get_logger("AstarPlanner"), "AstarPlanner finished reconstructing path");
    return path;
}
