"""
Differential Drive Robot Simulator

This module implements a simple 2D simulator for a differential drive robot
using Euler integration for state updates.
"""

import numpy as np
from typing import Tuple


class DifferentialDriveRobot:
    """
    Simulates a differential drive robot in 2D space.
    
    The robot uses the standard kinematic model:
        x_dot = v * cos(θ)
        y_dot = v * sin(θ)
        θ_dot = ω
    
    Attributes:
        x (float): X position in meters
        y (float): Y position in meters
        theta (float): Heading angle in radians
        wheelbase (float): Distance between wheels in meters
        history_x (list): History of X positions
        history_y (list): History of Y positions
        history_theta (list): History of heading angles
        time_history (list): History of timestamps
    """
    
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, 
                 wheelbase: float = 0.16):
        """
        Initialize the differential drive robot.
        
        Args:
            x: Initial X position (meters)
            y: Initial Y position (meters)
            theta: Initial heading angle (radians)
            wheelbase: Distance between wheels (meters), default 0.16 for TurtleBot3
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.wheelbase = wheelbase
        
        # History for tracking and visualization
        self.history_x = [x]
        self.history_y = [y]
        self.history_theta = [theta]
        self.time_history = [0.0]
        self.current_time = 0.0
    
    def update(self, v: float, omega: float, dt: float = 0.05) -> None:
        """
        Update robot state using differential drive kinematics (Euler integration).
        
        Args:
            v: Linear velocity command (m/s)
            omega: Angular velocity command (rad/s)
            dt: Time step for integration (seconds)
        """
        # Euler integration of kinematic equations
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        
        # Normalize theta to [-pi, pi] to avoid angle wrapping issues
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
        # Update time and history
        self.current_time += dt
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_theta.append(self.theta)
        self.time_history.append(self.current_time)
    
    def get_state(self) -> Tuple[float, float, float]:
        """
        Get the current robot state.
        
        Returns:
            Tuple of (x, y, theta) representing position and heading
        """
        return self.x, self.y, self.theta
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """
        Reset robot to a new initial state and clear history.
        
        Args:
            x: New X position (meters)
            y: New Y position (meters)
            theta: New heading angle (radians)
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.history_x = [x]
        self.history_y = [y]
        self.history_theta = [theta]
        self.time_history = [0.0]
        self.current_time = 0.0
    
    def get_history(self) -> Tuple[list, list, list, list]:
        """
        Get the complete state history.
        
        Returns:
            Tuple of (x_history, y_history, theta_history, time_history)
        """
        return self.history_x, self.history_y, self.history_theta, self.time_history
