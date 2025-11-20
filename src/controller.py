"""
Pure Pursuit Controller Module

Implements the Pure Pursuit algorithm for trajectory tracking
on differential drive robots.
"""

import numpy as np
from typing import Tuple, Optional
from .obstacle import ObstacleManager
from .local_planner import DWAPlanner


class PurePursuitController:
    """
    Pure Pursuit controller for path tracking.
    
    Pure Pursuit is a geometric path tracking algorithm that calculates
    the angular velocity command to steer the robot toward a lookahead
    point on the reference path.
    
    Algorithm:
        1. Find closest point on path to robot
        2. Find lookahead point at distance L ahead on path
        3. Calculate angle α to lookahead point
        4. Calculate angular velocity: ω = 2*v*sin(α)/L
    
    Attributes:
        L (float): Lookahead distance in meters
        v (float): Target linear velocity in m/s
        path_x (np.ndarray): X coordinates of reference path
        path_y (np.ndarray): Y coordinates of reference path
        current_idx (int): Current index on path (for efficiency)
        obstacle_manager (ObstacleManager): Obstacle manager for collision checking
        local_planner (DWAPlanner): DWA planner for local obstacle avoidance
        detection_radius (float): Radius for obstacle detection (meters)
        use_obstacle_avoidance (bool): Flag to enable obstacle avoidance
    """
    
    def __init__(self, lookahead_distance: float = 0.5, 
                 linear_velocity: float = 0.2):
        """
        Initialize Pure Pursuit controller.
        
        Args:
            lookahead_distance: Distance to lookahead point in meters (default: 0.5)
            linear_velocity: Target linear velocity in m/s (default: 0.2)
        
        Raises:
            ValueError: If lookahead distance or velocity are non-positive
        """
        if lookahead_distance <= 0:
            raise ValueError("Lookahead distance must be positive")
        if linear_velocity <= 0:
            raise ValueError("Linear velocity must be positive")
        
        self.L = lookahead_distance
        self.v = linear_velocity
        self.path_x = None
        self.path_y = None
        self.current_idx = 0
        
        # Constants for goal detection
        self.goal_tolerance = 0.1  # meters
        self.max_angular_velocity = 2.84  # rad/s (TurtleBot3 limit)
        
        # Obstacle avoidance components
        self.obstacle_manager = ObstacleManager()
        self.local_planner = DWAPlanner(max_v=linear_velocity)
        self.local_planner.set_obstacles(self.obstacle_manager)
        self.detection_radius = 1.0  # meters
        self.use_obstacle_avoidance = False
        
        # Track current velocity for DWA
        self.current_v = 0.0
        self.current_omega = 0.0
    
    def set_path(self, path_x: np.ndarray, path_y: np.ndarray) -> None:
        """
        Set the reference path to follow.
        
        Args:
            path_x: X coordinates of path
            path_y: Y coordinates of path
        
        Raises:
            ValueError: If paths are empty or have different lengths
        """
        if len(path_x) == 0 or len(path_y) == 0:
            raise ValueError("Path cannot be empty")
        if len(path_x) != len(path_y):
            raise ValueError("Path X and Y must have same length")
        
        self.path_x = np.array(path_x)
        self.path_y = np.array(path_y)
        self.current_idx = 0
    
    def find_closest_point(self, robot_x: float, robot_y: float) -> int:
        """
        Find index of closest point on path to robot.
        
        Args:
            robot_x: Robot's X position
            robot_y: Robot's Y position
        
        Returns:
            Index of closest point on path
        """
        if self.path_x is None:
            raise RuntimeError("Path not set. Call set_path() first.")
        
        # Calculate distances to all path points
        distances = np.sqrt((self.path_x - robot_x)**2 + 
                           (self.path_y - robot_y)**2)
        
        return int(np.argmin(distances))
    
    def find_lookahead_point(self, robot_x: float, robot_y: float, 
                            start_idx: int) -> Tuple[int, float, float]:
        """
        Find point on path at lookahead distance ahead.
        
        Searches forward from start_idx to find the first point that is
        at least lookahead distance away from the robot.
        
        Args:
            robot_x: Robot's X position
            robot_y: Robot's Y position
            start_idx: Index to start search from
        
        Returns:
            Tuple of (index, x, y) of lookahead point
        """
        if self.path_x is None:
            raise RuntimeError("Path not set. Call set_path() first.")
        
        # Search forward from start_idx
        for i in range(start_idx, len(self.path_x)):
            dist = np.sqrt((self.path_x[i] - robot_x)**2 + 
                          (self.path_y[i] - robot_y)**2)
            
            if dist >= self.L:
                return i, float(self.path_x[i]), float(self.path_y[i])
        
        # If no point found at lookahead distance, return last point
        last_idx = len(self.path_x) - 1
        return last_idx, float(self.path_x[last_idx]), float(self.path_y[last_idx])
    
    def compute_control(self, robot_x: float, robot_y: float, 
                       robot_theta: float) -> Tuple[float, float]:
        """
        Compute control commands (v, ω) for current robot state.
        
        This is the main control loop that implements Pure Pursuit logic
        with obstacle avoidance integration. When obstacles are nearby,
        switches to DWA for local reactive avoidance.
        
        Args:
            robot_x: Robot's current X position (meters)
            robot_y: Robot's current Y position (meters)
            robot_theta: Robot's current heading angle (radians)
        
        Returns:
            Tuple of (v, omega) where:
                v: Linear velocity command (m/s)
                omega: Angular velocity command (rad/s)
        """
        if self.path_x is None:
            raise RuntimeError("Path not set. Call set_path() first.")
        
        # Find closest point on path (for progress tracking)
        closest_idx = self.find_closest_point(robot_x, robot_y)
        
        # Update current index (never go backwards)
        self.current_idx = max(self.current_idx, closest_idx)
        
        # Check if goal is reached
        dist_to_goal = np.sqrt((self.path_x[-1] - robot_x)**2 + 
                              (self.path_y[-1] - robot_y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            # Goal reached, stop the robot
            self.current_v = 0.0
            self.current_omega = 0.0
            return 0.0, 0.0
        
        # Check for nearby obstacles
        if self.has_nearby_obstacles(robot_x, robot_y):
            # Use DWA for local obstacle avoidance
            
            # Find lookahead point as goal for DWA
            lookahead_idx, goal_x, goal_y = self.find_lookahead_point(
                robot_x, robot_y, self.current_idx
            )
            
            # Compute control using DWA
            v, omega = self.local_planner.compute_control(
                robot_x, robot_y, robot_theta,
                self.current_v, self.current_omega,
                goal_x, goal_y,
                self.path_x, self.path_y
            )
        else:
            # Use Pure Pursuit for nominal path following
            
            # Find lookahead point
            lookahead_idx, target_x, target_y = self.find_lookahead_point(
                robot_x, robot_y, self.current_idx
            )
            
            # Calculate angle to lookahead point
            dx = target_x - robot_x
            dy = target_y - robot_y
            target_angle = np.arctan2(dy, dx)
            
            # Calculate steering angle (difference from heading)
            alpha = target_angle - robot_theta
            
            # Normalize alpha to [-pi, pi]
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            
            # Pure Pursuit control law: ω = 2*v*sin(α)/L
            omega = 2 * self.v * np.sin(alpha) / self.L
            
            # Clamp angular velocity to robot limits
            omega = np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity)
            
            v = self.v
        
        # Update current velocity for next iteration
        self.current_v = v
        self.current_omega = omega
        
        return v, omega
    
    def get_tracking_error(self, robot_x: float, robot_y: float) -> float:
        """
        Calculate cross-track error (distance from robot to path).
        
        Args:
            robot_x: Robot's X position
            robot_y: Robot's Y position
        
        Returns:
            Distance to closest point on path (meters)
        """
        if self.path_x is None:
            return 0.0
        
        closest_idx = self.find_closest_point(robot_x, robot_y)
        error = np.sqrt((self.path_x[closest_idx] - robot_x)**2 + 
                       (self.path_y[closest_idx] - robot_y)**2)
        
        return float(error)
    
    def set_obstacles(self, obstacle_manager: ObstacleManager) -> None:
        """
        Set obstacle manager for obstacle avoidance.
        
        Args:
            obstacle_manager: ObstacleManager instance with obstacles
        """
        self.obstacle_manager = obstacle_manager
        self.local_planner.set_obstacles(obstacle_manager)
        self.use_obstacle_avoidance = len(obstacle_manager) > 0
    
    def has_nearby_obstacles(self, robot_x: float, robot_y: float) -> bool:
        """
        Check if there are obstacles within detection radius.
        
        Args:
            robot_x: Robot's X position
            robot_y: Robot's Y position
        
        Returns:
            True if obstacles are nearby
        """
        if not self.use_obstacle_avoidance:
            return False
        
        nearby = self.obstacle_manager.get_obstacles_in_range(
            robot_x, robot_y, self.detection_radius
        )
        return len(nearby) > 0
