"""
Unit tests for obstacle module.
"""

import pytest
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.obstacle import Obstacle, ObstacleManager


class TestObstacle:
    """Tests for Obstacle class."""
    
    def test_obstacle_creation(self):
        """Test creating an obstacle."""
        obs = Obstacle(x=1.0, y=2.0, radius=0.5)
        assert obs.x == 1.0
        assert obs.y == 2.0
        assert obs.radius == 0.5
    
    def test_obstacle_invalid_radius(self):
        """Test that invalid radius raises error."""
        with pytest.raises(ValueError):
            Obstacle(x=0, y=0, radius=0)
        
        with pytest.raises(ValueError):
            Obstacle(x=0, y=0, radius=-0.5)
    
    def test_distance_to_point(self):
        """Test distance calculation from obstacle to point."""
        obs = Obstacle(x=0, y=0, radius=1.0)
        
        # Point outside obstacle
        dist = obs.distance_to_point(3.0, 0.0)
        assert abs(dist - 2.0) < 1e-6  # 3.0 - 1.0 = 2.0
        
        # Point on surface
        dist = obs.distance_to_point(1.0, 0.0)
        assert abs(dist) < 1e-6
        
        # Point inside obstacle
        dist = obs.distance_to_point(0.5, 0.0)
        assert dist < 0
    
    def test_is_point_inside(self):
        """Test point collision detection."""
        obs = Obstacle(x=0, y=0, radius=1.0)
        
        # Point inside
        assert obs.is_point_inside(0.0, 0.0)
        assert obs.is_point_inside(0.5, 0.0)
        
        # Point outside
        assert not obs.is_point_inside(2.0, 0.0)
        assert not obs.is_point_inside(1.5, 1.5)
        
        # Point on boundary (just inside)
        assert obs.is_point_inside(1.0, 0.0, safety_margin=0.1)
    
    def test_collides_with_circle(self):
        """Test circle-to-circle collision."""
        obs = Obstacle(x=0, y=0, radius=1.0)
        
        # Overlapping circles
        assert obs.collides_with_circle(1.5, 0.0, 0.6)
        
        # Touching circles
        assert obs.collides_with_circle(2.0, 0.0, 1.0)
        
        # Separated circles
        assert not obs.collides_with_circle(3.0, 0.0, 1.0)
    
    def test_repr(self):
        """Test string representation."""
        obs = Obstacle(x=1.5, y=2.5, radius=0.75)
        repr_str = repr(obs)
        assert 'Obstacle' in repr_str
        assert '1.50' in repr_str or '1.5' in repr_str


class TestObstacleManager:
    """Tests for ObstacleManager class."""
    
    def test_manager_creation(self):
        """Test creating empty manager."""
        manager = ObstacleManager()
        assert len(manager) == 0
    
    def test_add_obstacle(self):
        """Test adding obstacles."""
        manager = ObstacleManager()
        obs = Obstacle(x=1, y=1, radius=0.5)
        
        manager.add_obstacle(obs)
        assert len(manager) == 1
    
    def test_add_circular_obstacle(self):
        """Test adding obstacle by coordinates."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(x=2, y=3, radius=0.4)
        
        assert len(manager) == 1
        obs = list(manager)[0]
        assert obs.x == 2
        assert obs.y == 3
        assert obs.radius == 0.4
    
    def test_remove_all(self):
        """Test removing all obstacles."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 1, 0.5)
        manager.add_circular_obstacle(2, 2, 0.5)
        
        assert len(manager) == 2
        manager.remove_all()
        assert len(manager) == 0
    
    def test_get_obstacles_in_range(self):
        """Test finding obstacles within range."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 0, 0.3)   # Close
        manager.add_circular_obstacle(5, 0, 0.3)   # Far
        manager.add_circular_obstacle(2, 0, 0.3)   # Medium
        
        # Find obstacles within 2m of origin
        nearby = manager.get_obstacles_in_range(0, 0, 2.0)
        assert len(nearby) == 2  # Should find first two (distances 0.7 and 1.7)
    
    def test_check_point_collision(self):
        """Test point collision with any obstacle."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 1, 0.5)
        manager.add_circular_obstacle(3, 3, 0.5)
        
        # Collision with first obstacle
        assert manager.check_point_collision(1, 1)
        
        # Collision with second obstacle
        assert manager.check_point_collision(3, 3)
        
        # No collision
        assert not manager.check_point_collision(0, 0)
        assert not manager.check_point_collision(5, 5)
    
    def test_check_circle_collision(self):
        """Test circle collision with any obstacle."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(2, 2, 0.5)
        
        # Collision
        assert manager.check_circle_collision(2.8, 2, 0.5)
        
        # No collision
        assert not manager.check_circle_collision(5, 5, 0.5)
    
    def test_check_path_collision(self):
        """Test path collision detection."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(2, 0, 0.5)
        
        # Path that goes through obstacle
        path_x = np.array([0, 1, 2, 3, 4])
        path_y = np.array([0, 0, 0, 0, 0])
        
        has_collision, indices = manager.check_path_collision(path_x, path_y, robot_radius=0.2)
        assert has_collision
        assert 2 in indices  # Point at x=2 should collide
        
        # Path that avoids obstacle
        path_x = np.array([0, 1, 2, 3, 4])
        path_y = np.array([2, 2, 2, 2, 2])
        
        has_collision, indices = manager.check_path_collision(path_x, path_y, robot_radius=0.2)
        assert not has_collision
        assert len(indices) == 0
    
    def test_get_closest_obstacle(self):
        """Test finding closest obstacle."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 0, 0.3)
        manager.add_circular_obstacle(3, 0, 0.3)
        
        # Closest to first obstacle
        obs, dist = manager.get_closest_obstacle(0, 0)
        assert obs.x == 1
        assert abs(dist - 0.7) < 1e-6  # Distance 1.0 - radius 0.3
        
        # Closest to second obstacle
        obs, dist = manager.get_closest_obstacle(4, 0)
        assert obs.x == 3
        assert abs(dist - 0.7) < 1e-6
    
    def test_get_closest_obstacle_empty(self):
        """Test closest obstacle with no obstacles."""
        manager = ObstacleManager()
        result = manager.get_closest_obstacle(0, 0)
        assert result is None
    
    def test_get_minimum_clearance(self):
        """Test calculating minimum clearance along path."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(2, 0, 0.5)
        
        path_x = np.array([0, 1, 1.5, 3, 4])
        path_y = np.array([0, 0, 0, 0, 0])
        
        min_clearance = manager.get_minimum_clearance(path_x, path_y)
        # Closest point is at x=1.5, distance = 2-1.5=0.5, minus radius 0.5 = 0
        assert min_clearance >= 0
        assert min_clearance < 0.6
    
    def test_iteration(self):
        """Test iterating over obstacles."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 1, 0.3)
        manager.add_circular_obstacle(2, 2, 0.4)
        
        count = 0
        for obs in manager:
            count += 1
            assert isinstance(obs, Obstacle)
        
        assert count == 2
    
    def test_repr(self):
        """Test string representation of manager."""
        manager = ObstacleManager()
        manager.add_circular_obstacle(1, 1, 0.5)
        
        repr_str = repr(manager)
        assert 'ObstacleManager' in repr_str
        assert '1' in repr_str
