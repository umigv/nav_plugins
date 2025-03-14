import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Enhanced MPPI implementation for differential drive robot
class MPPI:
    def __init__(self, model_function, cost_function, horizon_length=10, num_samples=200, 
                 noise_sigma=1, lambda_=1.0, control_dims=2,filter_window_size=5):
        """
        Initialize MPPI controller
        
        Args:
            model_function: Function that predicts next state given current state and input
            cost_function: Function that computes stage cost for a given state and input
            horizon_length: Number of steps to look ahead (N)
            num_samples: Number of Monte Carlo rollouts (K)
            noise_sigma: Standard deviation of control perturbations
            lambda_: Temperature parameter for reweighting
            control_dims: Dimensionality of the control inputs
        """
        self.model = model_function
        self.cost_function = cost_function
        self.N = horizon_length
        self.K = num_samples
        self.sigma = noise_sigma
        self.lambda_ = lambda_
        self.control_dims = control_dims
        self.filter_window_size = filter_window_size
        
        # Initialize control sequence
        self.u = np.zeros((self.N, self.control_dims))

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        """Apply moving average filter for smoothing input sequence
        
        Args:
            xx: Input sequence to filter (N x dim)
            window_size: Size of the filter window
            
        Returns:
            xx_mean: Filtered sequence
        """
        b = np.ones(window_size)/window_size
        dim = xx.shape[1]
        xx_mean = np.zeros(xx.shape)

        for d in range(dim):
            xx_mean[:,d] = np.convolve(xx[:,d], b, mode="same")
            n_conv = math.ceil(window_size/2)
            xx_mean[0,d] *= window_size/n_conv
            for i in range(1, n_conv):
                xx_mean[i,d] *= window_size/(i+n_conv)
                xx_mean[-i,d] *= window_size/(i + n_conv - (window_size % 2))
        return xx_mean
    
    def update(self, current_state, target_point):
        """
        Run one iteration of MPPI control update
        
        Args:
            current_state: Current state of the system [x, y, theta]
            target_point: Current target waypoint [x, y]
            
        Returns:
            Optimal control input to apply [v, w]
        """
        # Generate random perturbations
        delta_u = np.random.normal(0, self.sigma, (self.K, self.N, self.control_dims))
        
        # Initialize costs for each rollout
        S = np.zeros(self.K)
        
        # Perform Monte Carlo rollouts
        for k in range(self.K):
            x_k = current_state.copy()  # Start in current state
            
            # Rollout over horizon
            for n in range(self.N):
                # Input = nominal + perturbation
                u_k_n = self.u[n] + delta_u[k, n]
                
                # Compute next state using model
                x_k = self.model(x_k, u_k_n)
                
                # Accumulate cost
                S[k] += self.cost_function(x_k, u_k_n, target_point)
        
        # Compute weights using softmax
        weights = np.exp(-1.0 / self.lambda_ * (S - np.min(S)))
        weights = weights / (np.sum(weights) + 1e-10)  # Normalize
        
        # Update control sequence using reward-weighted perturbations
        for n in range(self.N):
            weighted_perturbations = np.sum(weights[:, None] * delta_u[:, n, :], axis=0)
            self.u[n] += weighted_perturbations
        
        # Apply moving average filter to smooth the control sequence
        self.u = self._moving_average_filter(self.u, self.filter_window_size)

        # First control input to apply
        u_optimal = self.u[0].copy()
        
        # Shift control sequence (drop first control, duplicate last)
        self.u[:-1] = self.u[1:]
        self.u[-1] = self.u[-2]  # Duplicate the second-to-last control
        
        return u_optimal

# Differential drive kinematics
def diff_drive_model(state, control, dt=0.1):
    """
    Differential drive kinematics model
    
    Args:
        state: Current state [x, y, theta]
        control: Control inputs [v, w] (linear and angular velocity)
        dt: Time step
        
    Returns:
        next_state: [x', y', theta']
    """
    x, y, theta = state
    v, w = control
    
    # # Constrain control inputs (optional)
    # v = np.clip(v, -1.0, 1.0)
    # w = np.clip(w, -1.0, 1.0)
    
    # Update state
    next_x = x + v * np.cos(theta) * dt
    next_y = y + v * np.sin(theta) * dt
    next_theta = theta + w * dt
    
    return np.array([next_x, next_y, next_theta])

# Cost function
def waypoint_cost_function(state, control, target_point, Q=np.array([10.0, 10.0]), R=np.array([0.1,0.2])):
    """
    Cost function for waypoint tracking with control penalty
    
    Args:
        state: Current state [x, y, theta]
        control: Control inputs [v, w]
        target_point: Target waypoint [x, y]
        Q: Weights for position error
        R: Weights for control effort
        
    Returns:
        cost: Total stage cost
    """
    # Position error cost
    x, y, _ = state
    target_x, target_y = target_point
    
    pos_error = np.array([x - target_x, y - target_y])
    pos_cost = np.sum(Q * (pos_error ** 2))
    
    # Control cost (penalize large or rapidly changing controls)
    control_cost = np.sum(R * (control ** 2))
    
    # Total cost
    return pos_cost + control_cost

# Test with waypoint navigation
def run_waypoint_navigation(waypoints, max_steps=1000):
    # Initialize robot state [x, y, theta]
    initial_state = np.array([-5.0, 5.0, 0.0])
    current_state = initial_state.copy()
    
    # Initialize MPPI controller
    mppi = MPPI(model_function=diff_drive_model, 
                cost_function=waypoint_cost_function,
                horizon_length=20, 
                num_samples=100, 
                noise_sigma=1,
                lambda_=0.5)
    
    # Set up for visualization
    state_history = [current_state.copy()]
    target_history = []
    control_history = []
    
    # Waypoint navigation
    waypoint_idx = 0
    steps = 0
    waypoint_reached_threshold = 0.4
    
    while waypoint_idx < len(waypoints) and steps < max_steps:
        # Current target waypoint
        target_point = waypoints[waypoint_idx]
        target_history.append(target_point)
        
        # Get optimal control from MPPI
        u_optimal = mppi.update(current_state, target_point)
        control_history.append(u_optimal)
        
        # Apply control to system and get feedback
        current_state = diff_drive_model(current_state, u_optimal)
        state_history.append(current_state.copy())
        
        # Check if waypoint reached
        dist_to_waypoint = np.sqrt((current_state[0] - target_point[0])**2 + 
                                  (current_state[1] - target_point[1])**2)
        
        if dist_to_waypoint < waypoint_reached_threshold:
            print(f"Reached waypoint {waypoint_idx}: {target_point}")
            waypoint_idx += 1
        
        steps += 1
    
    print(f"Navigation completed in {steps} steps")
    
    # Visualize results
    plot_trajectory(state_history, waypoints, target_history)
    plot_control(control_history)
    
    return state_history, control_history
def plot_control(control):
    
    control_history = np.array(control)
    # Plot trajectory
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)  # Two subplots, one column

    axes[0].plot(control_history[:, 0], 'b-', label='Input 1')
    axes[0].set_ylabel("Control Input 1")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(control_history[:, 1], 'b-', label='Input 2')
    axes[1].set_ylabel("Control Input 2")
    axes[1].set_xlabel("Time Step")
    axes[1].legend()
    axes[1].grid(True)

    plt.suptitle("Control Inputs Over Time")
    plt.show()

def plot_trajectory(state_history, waypoints, target_history):
    plt.figure(figsize=(10, 8))
    
    # Convert state history to numpy array
    state_history = np.array(state_history)
    
    # Plot trajectory
    plt.plot(state_history[:, 0], state_history[:, 1], 'b-', label='Robot Trajectory')
    
    # Plot waypoints
    waypoints = np.array(waypoints)
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro', markersize=10, label='Waypoints')
    
    # Plot starting position
    plt.plot(state_history[0, 0], state_history[0, 1], 'go', markersize=10, label='Start')
    
    # Add circles for waypoint thresholds
    # for waypoint in waypoints:
    #     circle = Circle((waypoint[0], waypoint[1]), 0.1, fill=False, color='r', linestyle='--')
    #     plt.gca().add_patch(circle)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Differential Drive Robot Navigation with MPPI')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

# Define waypoints for testing
# waypoints = [
#     [1.0, 0.0],
#     [2.0, 0.0],
#     [3.0, 0.0],
#     [4.0, 0.0],
#     [4.0,1.0],
#     [4.0,2.0],
#     [4.0,3.0],
#     [4.0,4.0],
#     [3.0,4.0],
#     [2.0,4.0],
#     [1.0,4.0]
    
# ]
# Define parameters
num_straight = 5  # Waypoints for each straight segment
num_curve = 30  # Waypoints for the curve

# Define straight path segments (descending left and ascending right)
x_left = np.linspace(-5, -3, num_straight)
y_left = np.linspace(5, 0, num_straight)

x_right = np.linspace(3, 5, num_straight)
y_right = np.linspace(0, 5, num_straight)

# Define smooth U-turn using a semicircle (bottom curve)
theta = np.linspace(np.pi, 2*np.pi, num_curve)
radius = 3
x_curve = radius * np.cos(theta)
y_curve = radius * np.sin(theta) - radius  # Shift downward

# Concatenate all segments
x_path = np.concatenate([x_left, x_curve, x_right])
y_path = np.concatenate([y_left, y_curve, y_right])
waypoints = np.vstack((x_path, y_path)).T

# Run the simulation
#state_history, control_history = run_waypoint_navigation(waypoints)