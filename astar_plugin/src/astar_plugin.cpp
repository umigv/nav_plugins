#include "astar_plugin/astar_plugin.hpp"
#include "rclcpp/rclcpp.hpp"


using Coordinate2D = infra_interfaces::msg::Coordinate2D;

AstarPlugin::AstarPlugin()
{
}

AstarPlugin::~AstarPlugin()
{
}

// void AstarPlugin::astar_init(const pair<int, int>& start, const pair<int, int>& goal, std::function<bool(int)> drivable, infra_common::Costmap &costmap){ //done
//     for(int i = 0; i < costmap.GetWidth(); i++){
//         for(int j = 0; j < costmap.GetHeight(); j++){
//             bool drivable = true;
//             if (!drivable(costmap.GetCost(curr.x, curr.y)))
//             {
//                 drivable = false;
//             }
//             if(drivable) Point newpoint({i, j}, 1);
//             else(drivable) Point newpoint({i, j}, 0);
//         }
//     }
//     grid[start.first][start.second].g_cost = 0;
//     open.push(&grid[start.first][start.second]);
//     grid[start.first][start.second].queued = true;
// }

// pair<int, int> AstarPlugin::position_to_cell(const Coordinate2D current, const float resolution, const std::pair<float, float> origin){
//     return {(current.x-origin.x)/resolution, (current.y-origin.y)/resolution};
// }
// Coordinate2D AstarPlugin::cell_to_position(const pair<int, int> current, const float resolution, const std::pair<float, float> origin){
//     return {current.first*resolution+origin.x, current.second*resolution+origin.y};
// }

Point* AstarPlugin::astar_alg(const pair<int, int>& start, const pair<int, int>& goal){
    // astar_init(start, goal);
    
    // while(!open.empty()){
    //     Point *current = open.top();
    //     open.pop();
    //     current->visited = true;
    //     if(current->pos == goal) { // not sure
    //         return current;
    //     }

    //     vector<Point*> neighbors = find_neighbors(current);
    //     for(Point* neighbor : neighbors){
    //         double new_cost = g_cost_calculation(neighbor, current);
    //         if(neighbor->g_cost > (current->g_cost + new_cost)){
    //             neighbor->parent = current;
    //             neighbor->g_cost = current->g_cost + new_cost;
    //             neighbor->h_cost = h_cost_calculation(neighbor, goal);
    //         }
    //         if(!neighbor->queued){
    //             open.push(neighbor);
    //             neighbor->queued = true;
    //         }
    //     }
    // }
    // return nullptr;
}   


std::vector<Coordinate2D> AstarPlugin::FindPath(infra_common::Costmap costmap, 
    std::function<bool(int)> drivable,
    Coordinate2D start,
    Coordinate2D goal)
{
    RCLCPP_INFO(rclcpp::get_logger("AstarPlugin"), "AstarPlugin planning path");
    
    // Straight line planner: just goes up in a straight line until it reaches the goal
    // or hits an obstacle
    std::vector<Coordinate2D> path;
    std::pair<float, float> origin = costmap.GetOrigin();
    float resolution = costmap.GetResolution();
    std::pair<int, int> start_ = position_to_cell(start, resolution, origin);
    std::pair<int, int> goal_ = position_to_cell(goal, resolution, origin);
    
    return path;
}



// astar algorithm completed below 

// #include <cmath>
// #include <iostream>
// #include <vector>
// #include <queue>
// #include <memory>
// #include <algorithm>

// using namespace std;


// class Point {
//  public:
//     pair<int, int> pos;
//     double g_cost;
//     double h_cost;
//     int occupied;
//     bool visited; // is in closed set
//     bool queued; // is in open list
//     Point *parent;

//     Point(const pair<int, int> pos_, int occupied_) :
//         pos(pos_), occupied(occupied_), g_cost(1000), visited(false), queued(false),parent(nullptr) {};

//     double f_cost() const {
//         return g_cost + h_cost;
//     }
// };

// struct ComparePointsCost {
//     bool operator()(const Point* p1, const Point* p2) const {
//         return p1->f_cost() > p2->f_cost(); // Min-heap based on f_cost
//     }
// };

// // Point operator methods
// bool operator<(const Point& lhs, const Point& rhs) { return (lhs.f_cost() < rhs.f_cost()); }
// bool operator>(const Point& lhs, const Point& rhs) { return (rhs < lhs); }
// bool operator<=(const Point& lhs, const Point& rhs) { return !(lhs > rhs); }
// bool operator>=(const Point& lhs, const Point& rhs) { return !(lhs < rhs); }

// vector<vector<Point>> grid;
// std::priority_queue<Point*, std::vector<Point*>, ComparePointsCost> open;
// std::pair<double, double> goal;
// std::pair<double, double> start;

// bool is_cell_occupied(int pos1, int pos2){
//     return grid[pos1][pos2].occupied > 0;
// }

// bool is_in_grid(int pos1, int pos2){
//     return pos1 < grid.size() && pos2 < grid[0].size() && pos1 >= 0 && pos2 >= 0;
// }  

// vector<Point*> find_neighbors(Point* p) {
//     // what are the indices of the neighbors?
//     // note: should probably make sure we don't go out of bounds...
//     vector<Point*> neighbors;

//     for (int row = -1; row <= 1; row++) {
//         for (int col = -1; col <=1; col++) {
//             int posx = p->pos.first+row;
//             int posy = p->pos.second+col;

//             if(!is_in_grid(posx, posy) || (row == 0 && col == 0)){
//                 continue;
//             }
//             if(!is_cell_occupied(posx, posy) && !grid[posx][posy].visited){
//                 neighbors.push_back(&grid[posx][posy]);
//             }
//         }
//     }


//     return neighbors;
// }

// double h_cost_calculation(Point* current_point, const pair<int, int>& goal){
//     double intermediate = pow(current_point->pos.first - goal.first, 2) + pow(current_point->pos.second - goal.second, 2);
//     return sqrt(intermediate);
// }

// double g_cost_calculation(Point* current_point, Point* parent){
//     double intermediate = pow(current_point->pos.first - parent->pos.first, 2) + pow(current_point->pos.second - parent->pos.second, 2);
//     return sqrt(intermediate);
// }

// void astar_init(const pair<int, int>& start, const pair<int, int>& goal){ //done
//     grid[start.first][start.second].g_cost = 0;
//     open.push(&grid[start.first][start.second]);
//     grid[start.first][start.second].queued = true;
// }

// // concerns:
// // vectors for the grid 
// // privacy concerns for the point class
// // check x and y indexing order


// // returns nullptr if found otherwise returns start_ptr
// Point* astar_alg(const pair<int, int>& start, const pair<int, int>& goal){
//     astar_init(start, goal);
    
//     while(!open.empty()){
//         Point *current = open.top();
//         open.pop();
//         current->visited = true;
//         if(current->pos == goal) { // not sure
//             return current;
//         }

//         vector<Point*> neighbors = find_neighbors(current);
//         for(Point* neighbor : neighbors){
//             double new_cost = g_cost_calculation(neighbor, current);
//             if(neighbor->g_cost > (current->g_cost + new_cost)){
//                 neighbor->parent = current;
//                 neighbor->g_cost = current->g_cost + new_cost;
//                 neighbor->h_cost = h_cost_calculation(neighbor, goal);
//             }
//             if(!neighbor->queued){
//                 open.push(neighbor);
//                 neighbor->queued = true;
//             }
//         }
//     }
//     return nullptr;
// }   


// void tests() {
//     // Test Case 1: 3x3 grid, 1 obstacle
//     // Expected result: a path going over and around the obstacle if checking neighboors clockwise. A vector of length 5.
//     cout << "Test Case 1" << endl;
//     for (int i = 0; i < 3; i++){
//         vector<Point> rows;
//         for(int j = 0; j < 3; j++){
//             rows.push_back(Point({i, j}, 0));
//         }
//         grid.push_back(rows);
//     }
//     grid[1][1].occupied = 1;
//     pair<int, int> start_pos_1 = {0, 0}; // top left
//     pair<int, int> end_pos_1 = {2, 2}; // bottom right
//     Point* start = astar_alg(start_pos_1, end_pos_1);
//     while(start != nullptr){
//         cout << start->pos.first << " " << start->pos.second << endl;
//         start = start->parent;
//     }
// }

// int main() {
//     tests();
//     return 0;
// }