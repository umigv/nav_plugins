#include "mppi.hpp"
#include <iostream>
#include <vector>
#include <cmath>

// Note: For visualization, a C++ plotting library would be needed
// Options include: matplotlibcpp, gnuplot-iostream, or writing data to file
// and using external tools for visualization

int main() {
    // Define parameters
    int num_straight = 5;  // Waypoints for each straight segment
    int num_curve = 30;    // Waypoints for the curve
    
    // Define waypoints
    std::vector<Eigen::Vector2d> waypoints;
    
    // Define straight path segments (descending left and ascending right)
    double x_left_start = -5.0;
    double x_left_end = -3.0;
    double y_left_start = 5.0;
    double y_left_end = 0.0;
    
    double x_right_start = 3.0;
    double x_right_end = 5.0;
    double y_right_start = 0.0;
    double y_right_end = 5.0;
    
    // Left segment
    for (int i = 0; i < num_straight; ++i) {
        double t = static_cast<double>(i) / (num_straight - 1);
        double x = x_left_start + t * (x_left_end - x_left_start);
        double y = y_left_start + t * (y_left_end - y_left_start);
        waypoints.push_back(Eigen::Vector2d(x, y));
    }
    
    // U-turn curve (semicircle)
    double radius = 3.0;
    for (int i = 0; i < num_curve; ++i) {
        double theta = M_PI + i * M_PI / (num_curve - 1);
        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta) - radius;  // Shift downward
        waypoints.push_back(Eigen::Vector2d(x, y));
    }
    
    // Right segment
    for (int i = 0; i < num_straight; ++i) {
        double t = static_cast<double>(i) / (num_straight - 1);
        double x = x_right_start + t * (x_right_end - x_right_start);
        double y = y_right_start + t * (y_right_end - y_right_start);
        waypoints.push_back(Eigen::Vector2d(x, y));
    }
    
    // Print waypoints
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  [" << waypoints[i](0) << ", " << waypoints[i](1) << "]" << std::endl;
    }
    
    // Run simulation
    auto [state_history, control_history] = runWaypointNavigation(
        waypoints,
        1000, // asdihsaodij
        20,
        100,
        1,
        0.5,
        Eigen::Vector2d(10.0, 10.0),
        Eigen::Vector2d(0.1, 0.2)
    );
    
    // Print final state
    std::cout << "Final state: [" 
              << state_history.back()(0) << ", " 
              << state_history.back()(1) << ", " 
              << state_history.back()(2) << "]" << std::endl;
    
    // For visualization in a real application, you could:
    // 1. Save trajectory to a file
    std::cout << "Writing trajectory to trajectory.csv..." << std::endl;
    FILE* f = fopen("trajectory.csv", "w");
    fprintf(f, "x,y,theta\n");
    for (const auto& state : state_history) {
        fprintf(f, "%f,%f,%f\n", state(0), state(1), state(2));
    }
    fclose(f);
    
    // 2. Save control inputs to a file
    std::cout << "Writing control inputs to controls.csv..." << std::endl;
    f = fopen("controls.csv", "w");
    fprintf(f, "v,w\n");
    for (const auto& control : control_history) {
        fprintf(f, "%f,%f\n", control(0), control(1));
    }
    fclose(f);
    
    std::cout << "Files saved. Use external tools to visualize the data." << std::endl;
    
    return 0;
}