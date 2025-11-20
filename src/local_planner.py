"""
Dynamic Window Approach (DWA) Local Planner

Implements local obstacle avoidance using the Dynamic Window Approach algorithm.
"""

import numpy as np
from typing import Tuple, List, Optional
from .obstacle import ObstacleManager


class DWAPlanner:
    """
    Dynamic Window Approach local planner for obstacle avoidance.
    
    DWA generates safe velocity commands by:
    1. Computing admissible velocities within robot's dynamic constraints
    2. Simulating trajectories for candidate velocities
    3. Evaluating trajectories based on goal, obstacles, and path
    4. Selecting optimal velocity command
    
    Attributes:
        max_v (float): Maximum linear velocity (m/s)
        min_v (float): Minimum linear velocity (m/s)
        max_omega (float): Maximum angular velocity (rad/s)
        max_accel (float): Maximum linear acceleration (m/s²)
        max_omega_accel (float): Maximum angular acceleration (rad/s²)
        v_resolution (float): Linear velocity sampling resolution
        omega_resolution (float): Angular velocity sampling resolution
        dt (float): Time step for trajectory prediction (s)
        predict_time (float): Trajectory prediction horizon (s)
        robot_radius (float): Robot radius for collision checking (m)
        obstacle_manager (ObstacleManager): Obstacle manager
    """
    
    def __init__(self,
                 max_v: float = 0.22,
                 min_v: float = 0.0,
                 max_omega: float = 2.84,
                 max_accel: float = 0.3,
                 max_omega_accel: float = 3.0,
                 v_resolution: float = 0.05,
                 omega_resolution: float = 0.2,
                 dt: float = 0.1,
                 predict_time: float = 1.5,  # Reduced for more responsive behavior
                 robot_radius: float = 0.105):
        """
        Initialize DWA planner.
        
        Args:
            max_v: Maximum linear velocity (m/s)
            min_v: Minimum linear velocity (m/s)
            max_omega: Maximum angular velocity (rad/s)
            max_accel: Maximum linear acceleration (m/s²)
            max_omega_accel: Maximum angular acceleration (rad/s²)
            v_resolution: Velocity sampling resolution
            omega_resolution: Angular velocity sampling resolution
            dt: Time step for prediction (s)
            predict_time: Prediction time horizon (s)
            robot_radius: Robot radius for collision checking (m)
        """
        self.max_v = max_v
        self.min_v = min_v
        self.max_omega = max_omega
        self.max_accel = max_accel
        self.max_omega_accel = max_omega_accel
        self.v_resolution = v_resolution
        self.omega_resolution = omega_resolution
        self.dt = dt
        self.predict_time = predict_time
        self.robot_radius = robot_radius
        
        self.obstacle_manager: Optional[ObstacleManager] = None
        
        # Cost function weights - prioritize goal reaching
        self.goal_weight = 1.0  # Increased to prioritize reaching goal
        self.obstacle_weight = 0.2  # Reduced - only avoid true collisions
        self.velocity_weight = 0.1  # Reduced
        self.path_weight = 0.3  # Keep moderate to stay on path
    
    def set_obstacles(self, obstacle_manager: ObstacleManager) -> None:
        """
        Set obstacle manager for collision checking.
        
        Args:
            obstacle_manager: ObstacleManager instance
        """
        self.obstacle_manager = obstacle_manager
    
    def calculate_dynamic_window(self, current_v: float, current_omega: float) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window of admissible velocities.
        
        The dynamic window is constrained by:
        1. Robot velocity limits
        2. Robot acceleration limits (what can be achieved in one time step)
        
        Args:
            current_v: Current linear velocity (m/s)
            current_omega: Current angular velocity (rad/s)
        
        Returns:
            Tuple of (v_min, v_max, omega_min, omega_max)
        """
        # Velocity limits
        v_range = [self.min_v, self.max_v]
        omega_range = [-self.max_omega, self.max_omega]
        
        # Dynamic constraints based on acceleration
        dv = self.max_accel * self.dt
        domega = self.max_omega_accel * self.dt
        
        dynamic_v_range = [current_v - dv, current_v + dv]
        dynamic_omega_range = [current_omega - domega, current_omega + domega]
        
        # Intersection of velocity limits and dynamic constraints
        v_min = max(v_range[0], dynamic_v_range[0])
        v_max = min(v_range[1], dynamic_v_range[1])
        omega_min = max(omega_range[0], dynamic_omega_range[0])
        omega_max = min(omega_range[1], dynamic_omega_range[1])
        
        return v_min, v_max, omega_min, omega_max
    
    def predict_trajectory(self, x: float, y: float, theta: float,
                          v: float, omega: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict robot trajectory for given velocity commands.
        
        Args:
            x: Initial X position
            y: Initial Y position
            theta: Initial heading angle
            v: Linear velocity command
            omega: Angular velocity command
        
        Returns:
            Tuple of (x_traj, y_traj, theta_traj) arrays
        """
        steps = int(self.predict_time / self.dt)
        x_traj = np.zeros(steps)
        y_traj = np.zeros(steps)
        theta_traj = np.zeros(steps)
        
        # Initial state
        x_current, y_current, theta_current = x, y, theta
        
        for i in range(steps):
            # Store current state
            x_traj[i] = x_current
            y_traj[i] = y_current
            theta_traj[i] = theta_current
            
            # Update state using unicycle model
            x_current += v * np.cos(theta_current) * self.dt
            y_current += v * np.sin(theta_current) * self.dt
            theta_current += omega * self.dt
        
        return x_traj, y_traj, theta_traj
    
    def check_trajectory_collision(self, x_traj: np.ndarray, 
                                   y_traj: np.ndarray) -> bool:
        """
        Check if trajectory collides with obstacles.
        
        Args:
            x_traj: X coordinates of trajectory
            y_traj: Y coordinates of trajectory
        
        Returns:
            True if trajectory collides with any obstacle
        """
        if self.obstacle_manager is None:
            return False
        
        for i in range(len(x_traj)):
            if self.obstacle_manager.check_circle_collision(
                x_traj[i], y_traj[i], self.robot_radius
            ):
                return True
        
        return False
    
    def calculate_goal_cost(self, x_traj: np.ndarray, y_traj: np.ndarray,
                           goal_x: float, goal_y: float) -> float:
        """
        Calculate cost based on distance to goal.
        
        Lower cost for trajectories that get closer to goal.
        
        Args:
            x_traj: X coordinates of trajectory
            y_traj: Y coordinates of trajectory
            goal_x: Goal X coordinate
            goal_y: Goal Y coordinate
        
        Returns:
            Goal cost (normalized)
        """
        # Distance from end of trajectory to goal
        final_x, final_y = x_traj[-1], y_traj[-1]
        dist_to_goal = np.sqrt((final_x - goal_x)**2 + (final_y - goal_y)**2)
        
        # Normalize (smaller distance = lower cost)
        return dist_to_goal
    
    def calculate_obstacle_cost(self, x_traj: np.ndarray, 
                               y_traj: np.ndarray) -> float:
        """
        Calculate cost based on proximity to obstacles.
        
        Lower cost for trajectories that maintain larger clearance.
        
        Args:
            x_traj: X coordinates of trajectory
            y_traj: Y coordinates of trajectory
        
        Returns:
            Obstacle cost (normalized)
        """
        if self.obstacle_manager is None or len(self.obstacle_manager) == 0:
            return 0.0
        
        min_clearance = float('inf')
        
        # Find minimum clearance along trajectory
        for i in range(len(x_traj)):
            result = self.obstacle_manager.get_closest_obstacle(x_traj[i], y_traj[i])
            if result:
                _, dist = result
                min_clearance = min(min_clearance, dist)
        
        # High cost if clearance is small
        # Use inverse with saturation
        if min_clearance < 0.05:  # Very close
            return 10.0
        elif min_clearance > 1.0:  # Far enough
            return 0.0
        else:
            return 1.0 / min_clearance
    
    def calculate_velocity_cost(self, v: float) -> float:
        """
        Calculate cost based on velocity.
        
        Prefer higher forward velocities (efficient motion).
        
        Args:
            v: Linear velocity
        
        Returns:
            Velocity cost (normalized)
        """
        # Prefer faster motion (lower cost for higher velocity)
        return (self.max_v - v) / self.max_v
    
    def calculate_path_alignment_cost(self, x_traj: np.ndarray, y_traj: np.ndarray,
                                     path_x: Optional[np.ndarray],
                                     path_y: Optional[np.ndarray]) -> float:
        """
        Calculate cost based on alignment with global path.
        
        Lower cost for trajectories that stay close to planned path.
        
        Args:
            x_traj: X coordinates of trajectory
            y_traj: Y coordinates of trajectory
            path_x: Global path X coordinates (optional)
            path_y: Global path Y coordinates (optional)
        
        Returns:
            Path alignment cost
        """
        if path_x is None or path_y is None or len(path_x) == 0:
            return 0.0
        
        # Calculate average distance from trajectory to path
        total_dist = 0.0
        
        for i in range(len(x_traj)):
            # Find closest point on path
            distances = np.sqrt((path_x - x_traj[i])**2 + (path_y - y_traj[i])**2)
            min_dist = np.min(distances)
            total_dist += min_dist
        
        avg_dist = total_dist / len(x_traj)
        return avg_dist
    
    def compute_control(self, x: float, y: float, theta: float,
                       current_v: float, current_omega: float,
                       goal_x: float, goal_y: float,
                       path_x: Optional[np.ndarray] = None,
                       path_y: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """
        Compute optimal velocity command using DWA.
        
        Args:
            x: Current X position
            y: Current Y position
            theta: Current heading angle
            current_v: Current linear velocity
            current_omega: Current angular velocity
            goal_x: Goal X coordinate
            goal_y: Goal Y coordinate
            path_x: Global path X coordinates (optional)
            path_y: Global path Y coordinates (optional)
        
        Returns:
            Tuple of (v, omega) optimal velocity command
        """
        # Calculate dynamic window
        v_min, v_max, omega_min, omega_max = self.calculate_dynamic_window(
            current_v, current_omega
        )
        
        # Generate velocity samples
        v_samples = np.arange(v_min, v_max + self.v_resolution, self.v_resolution)
        omega_samples = np.arange(omega_min, omega_max + self.omega_resolution, 
                                 self.omega_resolution)
        
        best_v = 0.0
        best_omega = 0.0
        min_cost = float('inf')
        found_valid = False
        
        # Evaluate all velocity combinations
        for v in v_samples:
            for omega in omega_samples:
                # Predict trajectory
                x_traj, y_traj, theta_traj = self.predict_trajectory(
                    x, y, theta, v, omega
                )
                
                # Check collision
                if self.check_trajectory_collision(x_traj, y_traj):
                    continue  # Skip colliding trajectories
                
                # Found at least one valid trajectory
                found_valid = True
                
                # Calculate costs
                goal_cost = self.calculate_goal_cost(x_traj, y_traj, goal_x, goal_y)
                obstacle_cost = self.calculate_obstacle_cost(x_traj, y_traj)
                velocity_cost = self.calculate_velocity_cost(v)
                path_cost = self.calculate_path_alignment_cost(x_traj, y_traj, path_x, path_y)
                
                # Total weighted cost
                total_cost = (self.goal_weight * goal_cost +
                            self.obstacle_weight * obstacle_cost +
                            self.velocity_weight * velocity_cost +
                            self.path_weight * path_cost)
                
                # Update best
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_v = v
                    best_omega = omega
        
        # Fallback: if no valid trajectory found, try to at least rotate towards goal
        if not found_valid:
            # Calculate angle to goal
            dx = goal_x - x
            dy = goal_y - y
            angle_to_goal = np.arctan2(dy, dx)
            angle_diff = angle_to_goal - theta
            
            # Normalize to [-pi, pi]
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            
            # Command slow rotation toward goal
            best_v = 0.05  # Very slow forward motion
            best_omega = np.clip(2.0 * angle_diff, -1.0, 1.0)  # Gentle rotation
        
        return best_v, best_omega
