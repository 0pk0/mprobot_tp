"""
Unit tests for path smoothing module.
"""

import pytest
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.path_smoother import smooth_path, calculate_curvature


class TestPathSmoother:
    """Test cases for path smoothing functionality."""
    
    def test_smooth_path_basic(self):
        """Test basic path smoothing functionality."""
        waypoints = [(0, 0), (1, 0), (2, 0), (3, 0)]
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        
        # Should have more points than input
        assert len(smooth_x) > len(waypoints)
        assert len(smooth_y) > len(waypoints)
        
        # Start and end should be close to original
        assert abs(smooth_x[0] - waypoints[0][0]) < 0.01
        assert abs(smooth_y[0] - waypoints[0][1]) < 0.01
        assert abs(smooth_x[-1] - waypoints[-1][0]) < 0.01
        assert abs(smooth_y[-1] - waypoints[-1][1]) < 0.01
    
    def test_smooth_path_curved(self):
        """Test smoothing on a curved path."""
        waypoints = [(0, 0), (1, 1), (2, 0)]
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.05)
        
        assert len(smooth_x) > len(waypoints)
        
        # Check smoothness - consecutive points should be close
        for i in range(1, len(smooth_x)):
            dist = np.sqrt((smooth_x[i] - smooth_x[i-1])**2 + 
                          (smooth_y[i] - smooth_y[i-1])**2)
            assert dist < 0.2  # Maximum distance between consecutive points
    
    def test_smooth_path_insufficient_waypoints(self):
        """Test that error is raised with too few waypoints."""
        waypoints = [(0, 0)]
        with pytest.raises(ValueError):
            smooth_path(waypoints)
    
    def test_smooth_path_same_points(self):
        """Test handling of identical waypoints."""
        waypoints = [(1, 1), (1, 1)]
        smooth_x, smooth_y = smooth_path(waypoints)
        
        # Should return original points
        assert len(smooth_x) == len(waypoints)
        assert np.allclose(smooth_x, 1.0)
        assert np.allclose(smooth_y, 1.0)
    
    def test_smooth_path_spacing(self):
        """Test that sample spacing is respected."""
        waypoints = [(0, 0), (5, 0)]
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.5)
        
        # For a 5m straight line with 0.5m spacing, should have ~10 points
        assert 8 <= len(smooth_x) <= 12
    
    def test_calculate_curvature(self):
        """Test curvature calculation."""
        # Straight line should have near-zero curvature
        x = np.array([0, 1, 2, 3, 4])
        y = np.array([0, 0, 0, 0, 0])
        curvature = calculate_curvature(x, y)
        
        assert np.all(curvature < 0.01)
    
    def test_smooth_path_circle(self):
        """Test smoothing on circular waypoints."""
        # Create waypoints on a circle
        theta = np.linspace(0, np.pi, 5)
        waypoints = [(np.cos(t), np.sin(t)) for t in theta]
        
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        
        # All points should be roughly on unit circle
        radii = np.sqrt(smooth_x**2 + smooth_y**2)
        assert np.all(np.abs(radii - 1.0) < 0.2)  # Allow some deviation
