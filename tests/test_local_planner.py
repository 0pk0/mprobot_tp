"""
Unit tests for DWA local planner.
"""

import pytest
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.local_planner import DWAPlanner
from src.obstacle import ObstacleManager


class TestDWAPlanner:
    """Tests for DWA local planner."""
    
    def test_planner_creation(self):
        """Test creating DWA planner."""
        planner = DWAPlanner()
        assert planner.max_v == 0.22
        assert planner.max_omega == 2.84
        assert planner.obstacle_manager is None
    
    def test_set_obstacles(self):
        """Test setting obstacle manager."""
        planner = DWAPlanner()
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 1, 0.5)
        
        planner.set_obstacles(manager)
        assert planner.obstacle_manager == manager
    
    def test_calculate_dynamic_window(self):
        """Test dynamic window calculation."""
        planner = DWAPlanner(max_v=0.5, max_omega=2.0, max_accel=0.5, max_omega_accel=2.0, dt=0.1)
        
        # From rest
        v_min, v_max, omega_min, omega_max = planner.calculate_dynamic_window(0.0, 0.0)
        
        # Should be limited by acceleration
        assert v_min == 0.0
        assert v_max == 0.05  # 0.5 * 0.1
        assert omega_min == -0.2  # -2.0 * 0.1
        assert omega_max == 0.2  # 2.0 * 0.1
    
    def test_calculate_dynamic_window_at_speed(self):
        """Test dynamic window when already moving."""
        planner = DWAPlanner(max_v=0.5, max_omega=2.0, max_accel=0.5, max_omega_accel=2.0, dt=0.1)
        
        # Already moving
        v_min, v_max, omega_min, omega_max = planner.calculate_dynamic_window(0.3, 0.5)
        
        # Window centered around current velocity
        assert v_min == 0.25  # 0.3 - 0.05
        assert v_max == 0.35  # 0.3 + 0.05
        assert omega_min == 0.3  # 0.5 - 0.2
        assert omega_max == 0.7  # 0.5 + 0.2
    
    def test_predict_trajectory(self):
        """Test trajectory prediction."""
        planner = DWAPlanner(dt=0.1, predict_time=1.0)
        
        # Predict straight motion
        x_traj, y_traj, theta_traj = planner.predict_trajectory(
            x=0, y=0, theta=0, v=1.0, omega=0
        )
        
        assert len(x_traj) == 10  # 1.0 / 0.1 = 10 steps
        assert len(y_traj) == 10
        assert len(theta_traj) == 10
        
        # Check final position (moved forward 1m)
        assert abs(x_traj[-1] - 1.0) < 0.15  # Should be close to 1.0
        assert abs(y_traj[-1]) < 0.1  # Should stay near 0
    
    def test_predict_trajectory_turning(self):
        """Test trajectory prediction with turning."""
        planner = DWAPlanner(dt=0.1, predict_time=1.0)
        
        # Predict turning motion
        x_traj, y_traj, theta_traj = planner.predict_trajectory(
            x=0, y=0, theta=0, v=0.5, omega=1.0
        )
        
        # Should turn and move
        assert x_traj[-1] > 0  # Moved forward
        assert abs(y_traj[-1]) > 0  # Turned (y changed)
        assert abs(theta_traj[-1]) > 0  # Heading changed
    
    def test_check_trajectory_collision_no_obstacles(self):
        """Test collision checking with no obstacles."""
        planner = DWAPlanner()
        
        x_traj = np.array([0, 1, 2, 3])
        y_traj = np.array([0, 0, 0, 0])
        
        collision = planner.check_trajectory_collision(x_traj, y_traj)
        assert not collision
    
    def test_check_trajectory_collision_with_obstacle(self):
        """Test collision detection with obstacles."""
        planner = DWAPlanner()
        manager = ObstacleManager()
        manager.add_circular_obstacle(2, 0, 0.5)
        planner.set_obstacles(manager)
        
        # Trajectory that passes through obstacle
        x_traj = np.array([0, 1, 2, 3])
        y_traj = np.array([0, 0, 0, 0])
        
        collision = planner.check_trajectory_collision(x_traj, y_traj)
        assert collision
        
        # Trajectory that avoids obstacle
        x_traj = np.array([0, 1, 2, 3])
        y_traj = np.array([2, 2, 2, 2])
        
        collision = planner.check_trajectory_collision(x_traj, y_traj)
        assert not collision
    
    def test_calculate_goal_cost(self):
        """Test goal cost calculation."""
        planner = DWAPlanner()
        
        # Trajectory ending at (5, 0), goal at (10, 0)
        x_traj = np.array([0, 1, 2, 3, 4, 5])
        y_traj = np.array([0, 0, 0, 0, 0, 0])
        
        cost = planner.calculate_goal_cost(x_traj, y_traj, goal_x=10, goal_y=0)
        assert abs(cost - 5.0) < 0.1  # Distance should be ~5
        
        # Trajectory closer to goal
        x_traj2 = np.array([5, 6, 7, 8, 9])
        cost2 = planner.calculate_goal_cost(x_traj2, y_traj[:5], goal_x=10, goal_y=0)
        assert cost2 < cost  # Closer = lower cost
    
    def test_calculate_obstacle_cost_no_obstacles(self):
        """Test obstacle cost with no obstacles."""
        planner = DWAPlanner()
        
        x_traj = np.array([0, 1, 2])
        y_traj = np.array([0, 0, 0])
        
        cost = planner.calculate_obstacle_cost(x_traj, y_traj)
        assert cost == 0.0
    
    def test_calculate_obstacle_cost_with_obstacles(self):
        """Test obstacle cost with obstacles nearby."""
        planner = DWAPlanner()
        manager = ObstacleManager()
        manager.add_circular_obstacle(2, 2, 0.5)
        planner.set_obstacles(manager)
        
        # Trajectory far from obstacle
        x_traj = np.array([  0, 1, 2])
        y_traj = np.array([0, 0, 0])
        cost_far = planner.calculate_obstacle_cost(x_traj, y_traj)
        
        # Trajectory close to obstacle
        x_traj2 = np.array([2, 2, 2])
        y_traj2 = np.array([1, 1.2, 1.4])
        cost_close = planner.calculate_obstacle_cost(x_traj2, y_traj2)
        
        assert cost_close > cost_far  # Closer = higher cost
    
    def test_calculate_velocity_cost(self):
        """Test velocity cost calculation."""
        planner = DWAPlanner(max_v=1.0)
        
        # Higher velocity should have lower cost
        cost_high = planner.calculate_velocity_cost(0.9)
        cost_low = planner.calculate_velocity_cost(0.1)
        
        assert cost_low > cost_high
    
    def test_compute_control_no_obstacles(self):
        """Test control computation without obstacles."""
        planner = DWAPlanner()
        
        # Simple case: move toward goal directly ahead
        v, omega = planner.compute_control(
            x=0, y=0, theta=0,
            current_v=0, current_omega=0,
            goal_x=5, goal_y=0
        )
        
        # Should command forward motion
        assert v > 0
        assert abs(omega) < 0.5  # Should be small (going straight)
    
    def test_compute_control_with_path(self):
        """Test control computation with reference path."""
        planner = DWAPlanner()
        
        path_x = np.array([0, 1, 2, 3, 4, 5])
        path_y = np.array([0, 0, 0, 0, 0, 0])
        
        v, omega = planner.compute_control(
            x=0, y=0, theta=0,
            current_v=0, current_omega=0,
            goal_x=5, goal_y=0,
            path_x=path_x, path_y=path_y
        )
        
        assert v > 0
        assert isinstance(omega, float)
    
    def test_compute_control_avoids_obstacles(self):
        """Test that DWA avoids obstacles."""
        planner = DWAPlanner(predict_time=1.0, dt=0.1)
        manager = ObstacleManager()
        # Place obstacle directly ahead
        manager.add_circular_obstacle(1, 0, 0.3)
        planner.set_obstacles(manager)
        
        v, omega = planner.compute_control(
            x=0, y=0, theta=0,
            current_v=0, current_omega=0,
            goal_x=5, goal_y=0
        )
        
        # Should still command some motion (not stuck)
        # Might turn or slow down to avoid
        assert v >= 0
        assert isinstance(omega, float)
