#pragma once

#include <Eigen/Dense>
#include <functional>
#include <random>
#include <vector>

class MPPI {
public:
    /**
     * Initialize MPPI controller
     * 
     * @param model_function Function that predicts next state given current state and input
     * @param cost_function Function that computes stage cost for a given state and input
     * @param horizon_length Number of steps to look ahead (N)
     * @param num_samples Number of Monte Carlo rollouts (K)
     * @param noise_sigma Standard deviation of control perturbations
     * @param lambda Temperature parameter for reweighting
     * @param control_dims Dimensionality of the control inputs
     * @param filter_window_size Size of the moving average filter window
     */
    MPPI(
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> model_function,
        std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&)> cost_function,
        int horizon_length = 10,
        int num_samples = 200,
        double noise_sigma = 1.0,
        double lambda = 1.0,
        int control_dims = 2,
        int filter_window_size = 5
    );

    /**
     * Run one iteration of MPPI control update
     * 
     * @param current_state Current state of the system [x, y, theta]
     * @param target_point Current target waypoint [x, y]
     * @return Optimal control input to apply [v, w]
     */
    Eigen::VectorXd update(const Eigen::VectorXd& current_state, const Eigen::VectorXd& target_point);

private:
    /**
     * Apply moving average filter for smoothing input sequence
     * 
     * @param xx Input sequence to filter (N x dim)
     * @param window_size Size of the filter window
     * @return Filtered sequence
     */
    Eigen::MatrixXd movingAverageFilter(const Eigen::MatrixXd& xx, int window_size);

    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> model;
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&)> cost_function;
    int N;                  // Horizon length
    int K;                  // Number of samples
    double sigma;           // Noise standard deviation
    double lambda_;         // Temperature parameter
    int control_dims;       // Control dimensions
    int filter_window_size; // Filter window size
    Eigen::MatrixXd u;      // Control sequence (N x control_dims)
    std::mt19937 rng;       // Random number generator
};

/**
 * Differential drive kinematics model
 * 
 * @param state Current state [x, y, theta]
 * @param control Control inputs [v, w] (linear and angular velocity)
 * @param dt Time step
 * @return Next state [x', y', theta']
 */
Eigen::VectorXd diffDriveModel(const Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt = 0.1);

/**
 * Cost function for waypoint tracking with control penalty
 * 
 * @param state Current state [x, y, theta]
 * @param control Control inputs [v, w]
 * @param target_point Target waypoint [x, y]
 * @param Q Weights for position error
 * @param R Weights for control effort
 * @return Total stage cost
 */
double waypointCostFunction(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    const Eigen::VectorXd& target_point,
    const Eigen::Vector2d& Q = Eigen::Vector2d(10.0, 10.0),
    const Eigen::Vector2d& R = Eigen::Vector2d(0.1, 0.2)
);

/**
 * Run waypoint navigation simulation
 * 
 * @param waypoints Vector of waypoints
 * @param max_steps Maximum number of simulation steps
 * @return Pair of state history and control history
 */
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> runWaypointNavigation(
    const std::vector<Eigen::Vector2d>& waypoints,
    int max_steps,
    double horizon_length,
    double num_samples,
    double noise_sigma,
    double lambda,
    Eigen::Vector2d Q,
    Eigen::Vector2d R
);