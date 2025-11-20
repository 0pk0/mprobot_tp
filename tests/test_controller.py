"""
Unit tests for Pure Pursuit controller.
"""

import pytest
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.controller import PurePursuitController


class TestPurePursuitController:
    """Test cases for Pure Pursuit controller."""
    
    def test_controller_initialization(self):
        """Test controller initialization."""
        controller = PurePursuitController(lookahead_distance=0.5, linear_velocity=0.2)
        assert controller.L == 0.5
        assert controller.v == 0.2
        assert controller.path_x is None
        assert controller.path_y is None
    
    def test_controller_invalid_parameters(self):
        """Test that invalid parameters raise errors."""
        with pytest.raises(ValueError):
            PurePursuitController(lookahead_distance=-0.5, linear_velocity=0.2)
        
        with pytest.raises(ValueError):
            PurePursuitController(lookahead_distance=0.5, linear_velocity=-0.2)
    
    def test_set_path(self):
        """Test setting reference path."""
        controller = PurePursuitController()
        path_x = np.array([0, 1, 2, 3])
        path_y = np.array([0, 0, 0, 0])
        
        controller.set_path(path_x, path_y)
        assert np.array_equal(controller.path_x, path_x)
        assert np.array_equal(controller.path_y, path_y)
    
    def test_set_path_invalid(self):
        """Test that invalid paths raise errors."""
        controller = PurePursuitController()
        
        # Empty path
        with pytest.raises(ValueError):
            controller.set_path(np.array([]), np.array([]))
        
        # Mismatched lengths
        with pytest.raises(ValueError):
            controller.set_path(np.array([0, 1]), np.array([0]))
    
    def test_find_closest_point(self):
        """Test finding closest point on path."""
        controller = PurePursuitController()
        path_x = np.array([0, 1, 2, 3, 4])
        path_y = np.array([0, 0, 0, 0, 0])
        controller.set_path(path_x, path_y)
        
        # Robot at (1.5, 0.5) should be closest to index 1 or 2
        closest_idx = controller.find_closest_point(1.5, 0.5)
        assert closest_idx in [1, 2]
        
        # Robot at (4.5, 0) should be closest to last point
        closest_idx = controller.find_closest_point(4.5, 0)
        assert closest_idx == 4
    
    def test_find_lookahead_point(self):
        """Test finding lookahead point."""
        controller = PurePursuitController(lookahead_distance=1.0)
        path_x = np.array([0, 1, 2, 3, 4, 5])
        path_y = np.array([0, 0, 0, 0, 0, 0])
        controller.set_path(path_x, path_y)
        
        # Robot at origin, lookahead should be around x=1
        idx, x, y = controller.find_lookahead_point(0, 0, 0)
        assert abs(x - 1.0) < 0.5
    
    def test_compute_control_straight_path(self):
        """Test control computation on straight path."""
        controller = PurePursuitController(lookahead_distance=0.5, linear_velocity=0.2)
        path_x = np.array([0, 1, 2, 3, 4, 5])
        path_y = np.array([0, 0, 0, 0, 0, 0])
        controller.set_path(path_x, path_y)
        
        # Robot aligned with path
        v, omega = controller.compute_control(0, 0, 0)
        assert v == 0.2
        assert abs(omega) < 0.1  # Should be nearly straight
    
    def test_compute_control_goal_reached(self):
        """Test that controller stops when goal is reached."""
        controller = PurePursuitController()
        path_x = np.array([0, 1, 2])
        path_y = np.array([0, 0, 0])
        controller.set_path(path_x, path_y)
        
        # Robot very close to goal
        v, omega = controller.compute_control(2.05, 0, 0)
        assert v == 0.0
        assert omega == 0.0
    
    def test_compute_control_turning(self):
        """Test control when robot needs to turn."""
        controller = PurePursuitController(lookahead_distance=0.5, linear_velocity=0.2)
        path_x = np.array([0, 1, 2, 3])
        path_y = np.array([0, 1, 2, 3])  # Diagonal path
        controller.set_path(path_x, path_y)
        
        # Robot at origin facing east (0 radians), should turn toward diagonal
        v, omega = controller.compute_control(0, 0, 0)
        assert v == 0.2
        assert omega > 0  # Should turn left (positive omega)
    
    def test_get_tracking_error(self):
        """Test tracking error calculation."""
        controller = PurePursuitController()
        path_x = np.array([0, 1, 2, 3])
        path_y = np.array([0, 0, 0, 0])
        controller.set_path(path_x, path_y)
        
        # Robot at (1, 1) should be 1m away from path
        error = controller.get_tracking_error(1, 1)
        assert abs(error - 1.0) < 0.01
        
        # Robot on path should have near-zero error
        error = controller.get_tracking_error(1, 0)
        assert error < 0.01
    
    def test_angular_velocity_limits(self):
        """Test that angular velocity is clamped to limits."""
        controller = PurePursuitController(lookahead_distance=0.1, linear_velocity=0.2)
        path_x = np.array([0, 0, 0])
        path_y = np.array([0, 1, 2])  # Path perpendicular to robot
        controller.set_path(path_x, path_y)
        
        # Robot facing wrong direction should produce large omega, but clamped
        v, omega = controller.compute_control(0, 0, 0)  # Facing east
        assert abs(omega) <= controller.max_angular_velocity
