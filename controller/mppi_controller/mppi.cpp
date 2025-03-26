#include "mppi.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

MPPI::MPPI(
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> model_function,
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&)> cost_function,
    int horizon_length,
    int num_samples,
    double noise_sigma,
    double lambda,
    int control_dims,
    int filter_window_size
) : model(model_function),
    cost_function(cost_function),
    N(horizon_length),
    K(num_samples),
    sigma(noise_sigma),
    lambda_(lambda),
    control_dims(control_dims),
    filter_window_size(filter_window_size),
    u(Eigen::MatrixXd::Zero(horizon_length, control_dims)),
    rng(std::random_device{}())
{
}

Eigen::MatrixXd MPPI::movingAverageFilter(const Eigen::MatrixXd& xx, int window_size) {
    // Create filter weights
    Eigen::VectorXd b = Eigen::VectorXd::Ones(window_size) / window_size;
    int dim = xx.cols();
    Eigen::MatrixXd xx_mean = Eigen::MatrixXd::Zero(xx.rows(), xx.cols());

    for (int d = 0; d < dim; ++d) {
        // Extract column
        Eigen::VectorXd col = xx.col(d);
        Eigen::VectorXd filtered = Eigen::VectorXd::Zero(col.size());
        
        // Perform convolution (simplified version of numpy's convolve with 'same' mode)
        for (int i = 0; i < col.size(); ++i) {
            double sum = 0.0;
            int count = 0;
            
            // Apply filter centered at position i
            for (int j = 0; j < window_size; ++j) {
                int idx = i - (window_size / 2) + j;
                if (idx >= 0 && idx < col.size()) {
                    sum += col(idx) * b(j);
                    count++;
                }
            }
            
            // Normalize by actual number of elements used
            if (count > 0) {
                filtered(i) = sum * window_size / count;
            }
        }
        
        xx_mean.col(d) = filtered;
    }
    
    return xx_mean;
}

Eigen::VectorXd MPPI::update(const Eigen::VectorXd& current_state, const Eigen::VectorXd& target_point) {
    // Generate random perturbations
    std::normal_distribution<double> dist(0.0, sigma);
    std::vector<Eigen::MatrixXd> delta_u(K, Eigen::MatrixXd::Zero(N, control_dims));
    
    for (int k = 0; k < K; ++k) {
        for (int n = 0; n < N; ++n) {
            for (int d = 0; d < control_dims; ++d) {
                delta_u[k](n, d) = dist(rng);
            }
        }
    }
    
    // Initialize costs for each rollout
    Eigen::VectorXd S = Eigen::VectorXd::Zero(K);
    
    // Perform Monte Carlo rollouts
    for (int k = 0; k < K; ++k) {
        Eigen::VectorXd x_k = current_state; // Start in current state
        
        // Rollout over horizon
        for (int n = 0; n < N; ++n) {
            // Input = nominal + perturbation
            Eigen::VectorXd u_k_n(control_dims);
            for (int d = 0; d < control_dims; ++d) {
                u_k_n(d) = u(n, d) + delta_u[k](n, d);
            }
            
            // Compute next state using model
            Eigen::VectorXd previous_state = x_k;
            x_k = model(x_k, u_k_n);
            
            // Accumulate cost
            S(k) += cost_function(previous_state, x_k, u_k_n, target_point);
        }
    }
    
    // Compute weights using softmax
    double min_cost = S.minCoeff();
    Eigen::VectorXd weights = (-1.0 / lambda_ * (S.array() - min_cost)).exp();
    weights /= weights.sum() + 1e-10; // Normalize
    
    // Update control sequence using reward-weighted perturbations
    for (int n = 0; n < N; ++n) {
        Eigen::VectorXd weighted_perturbation = Eigen::VectorXd::Zero(control_dims);
        
        for (int k = 0; k < K; ++k) {
            for (int d = 0; d < control_dims; ++d) {
                weighted_perturbation(d) += weights(k) * delta_u[k](n, d);
            }
        }
        
        for (int d = 0; d < control_dims; ++d) {
            u(n, d) += weighted_perturbation(d);
        }
    }
    
    // Apply moving average filter to smooth the control sequence
    u = movingAverageFilter(u, filter_window_size);
    
    // First control input to apply
    Eigen::VectorXd u_optimal(control_dims);
    for (int d = 0; d < control_dims; ++d) {
        u_optimal(d) = u(0, d);
    }
    
    // Shift control sequence (drop first control, duplicate last)
    for (int n = 0; n < N - 1; ++n) {
        u.row(n) = u.row(n + 1);
    }
    u.row(N - 1) = u.row(N - 2); // Duplicate the second-to-last control
    
    return u_optimal;
}

Eigen::VectorXd diffDriveModel(const Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt) {
    double x = state(0);
    double y = state(1);
    double theta = state(2);
    
    double v = control(0);
    double w = control(1);
    
    // Optional: Constrain control inputs
    v = std::clamp(v, -1.0, 1.0);
    w = std::clamp(w, -2.0, 2.0);
    
    // Update state
    double next_x = x + v * std::cos(theta) * dt;
    double next_y = y + v * std::sin(theta) * dt;
    double next_theta = theta + w * dt;
    
    Eigen::VectorXd next_state(3);
    next_state << next_x, next_y, next_theta;
    
    return next_state;
}

double waypointCostFunction(
    const Eigen::VectorXd& previous_state,
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    const Eigen::VectorXd& target_point,
    const Eigen::Vector2d& Q,
    const Eigen::Vector2d& R
) {
    // Position error cost
    double x = state(0);
    double y = state(1);
    
    double target_x = target_point(0);
    double target_y = target_point(1);

    double previous_theta = previous_state(2);
    double current_theta = state(2);
    
    Eigen::Vector2d pos_error;
    pos_error << x - target_x, y - target_y;
    
    double pos_cost = (Q.array() * pos_error.array().square()).sum();
    
    // Control cost (penalize large or rapidly changing controls)
    double control_cost = (R.array() * control.array().square()).sum();

    double rotation_cost = (Q(0) * std::abs(current_theta - previous_theta));

    // Total cost
    return pos_cost + control_cost + rotation_cost;
}

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> runWaypointNavigation(
    const std::vector<Eigen::Vector2d>& waypoints,
    int max_steps = 1000,
    double horizon_length = 20,
    double num_samples = 100,
    double noise_sigma = 1,
    double lambda = 0.5,
    Eigen::Vector2d Q = Eigen::Vector2d(10.0, 10.0),
    Eigen::Vector2d R = Eigen::Vector2d(0.1, 0.2)
) {
    // Initialize robot state [x, y, theta]
    Eigen::VectorXd initial_state(3);
    initial_state << -5.0, 5.0, 0.0;
    Eigen::VectorXd current_state = initial_state;
    
    // Create wrapper lambda functions with matching signatures
    auto model_wrapper = [](const Eigen::VectorXd& state, const Eigen::VectorXd& control) -> Eigen::VectorXd {
        return diffDriveModel(state, control, 0.1); // Use default dt=0.1
    };

    auto cost_wrapper = [Q, R](const Eigen::VectorXd& previous_state, const Eigen::VectorXd& state, const Eigen::VectorXd& control, 
                        const Eigen::VectorXd& target) -> double {
        return waypointCostFunction(previous_state, state, control, target, Q, R);
    };

    // Initialize MPPI controller
    MPPI mppi(
        model_wrapper,
        cost_wrapper,
        horizon_length,
        num_samples,
        noise_sigma,
        lambda
    );
    
    // Set up for recording history
    std::vector<Eigen::VectorXd> state_history;
    std::vector<Eigen::VectorXd> control_history;
    
    state_history.push_back(current_state);
    
    // Waypoint navigation
    int waypoint_idx = 0;
    int steps = 0;
    double waypoint_reached_threshold = 0.4;
    
    while (waypoint_idx < waypoints.size() && steps < max_steps) {
        // Current target waypoint
        Eigen::VectorXd target_point(2);
        target_point << waypoints[waypoint_idx](0), waypoints[waypoint_idx](1);
        
        // Get optimal control from MPPI
        Eigen::VectorXd u_optimal = mppi.update(current_state, target_point);
        control_history.push_back(u_optimal);
        
        // Apply control to system and get feedback
        current_state = diffDriveModel(current_state, u_optimal);
        state_history.push_back(current_state);
        
        // Check if waypoint reached
        double dist_to_waypoint = std::sqrt(
            std::pow(current_state(0) - target_point(0), 2) +
            std::pow(current_state(1) - target_point(1), 2)
        );
        
        if (dist_to_waypoint < waypoint_reached_threshold) {
            std::cout << "Reached waypoint " << waypoint_idx << ": ["
                      << target_point(0) << ", " << target_point(1) << "]" << std::endl;
            waypoint_idx++;
        }
        
        steps++;
    }
    
    std::cout << "Navigation completed in " << steps << " steps" << std::endl;
    
    return {state_history, control_history};
}