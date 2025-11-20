"""
Obstacle Module

Provides obstacle representation and collision detection for path planning.
"""

import numpy as np
from typing import List, Tuple, Optional


class Obstacle:
    """
    Represents a circular obstacle in 2D space.
    
    Attributes:
        x (float): X coordinate of obstacle center (meters)
        y (float): Y coordinate of obstacle center (meters)
        radius (float): Obstacle radius (meters)
    """
    
    def __init__(self, x: float, y: float, radius: float):
        """
        Initialize a circular obstacle.
        
        Args:
            x: X coordinate of obstacle center
            y: Y coordinate of obstacle center
            radius: Obstacle radius (must be positive)
        
        Raises:
            ValueError: If radius is non-positive
        """
        if radius <= 0:
            raise ValueError("Obstacle radius must be positive")
        
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)
    
    def distance_to_point(self, x: float, y: float) -> float:
        """
        Calculate distance from obstacle surface to a point.
        
        Args:
            x: X coordinate of point
            y: Y coordinate of point
        
        Returns:
            Distance from obstacle surface to point (negative if inside)
        """
        center_dist = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return center_dist - self.radius
    
    def is_point_inside(self, x: float, y: float, safety_margin: float = 0.0) -> bool:
        """
        Check if a point is inside the obstacle.
        
        Args:
            x: X coordinate of point
            y: Y coordinate of point
            safety_margin: Additional safety margin around obstacle (meters)
        
        Returns:
            True if point is inside obstacle (including safety margin)
        """
        dist = self.distance_to_point(x, y)
        return dist < safety_margin
    
    def collides_with_circle(self, x: float, y: float, radius: float) -> bool:
        """
        Check if this obstacle collides with another circle.
        
        Args:
            x: X coordinate of circle center
            y: Y coordinate of circle center
            radius: Radius of circle
        
        Returns:
            True if circles overlap
        """
        center_dist = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return center_dist < (self.radius + radius)
    
    def __repr__(self) -> str:
        return f"Obstacle(x={self.x:.2f}, y={self.y:.2f}, r={self.radius:.2f})"


class ObstacleManager:
    """
    Manages a collection of obstacles and provides collision checking utilities.
    
    Attributes:
        obstacles (List[Obstacle]): List of obstacles
    """
    
    def __init__(self):
        """Initialize an empty obstacle manager."""
        self.obstacles: List[Obstacle] = []
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """
        Add an obstacle to the manager.
        
        Args:
            obstacle: Obstacle to add
        """
        self.obstacles.append(obstacle)
    
    def add_circular_obstacle(self, x: float, y: float, radius: float) -> None:
        """
        Add a circular obstacle by coordinates.
        
        Args:
            x: X coordinate of obstacle center
            y: Y coordinate of obstacle center
            radius: Obstacle radius
        """
        self.add_obstacle(Obstacle(x, y, radius))
    
    def remove_all(self) -> None:
        """Remove all obstacles."""
        self.obstacles.clear()
    
    def get_obstacles_in_range(self, x: float, y: float, 
                               range_dist: float) -> List[Obstacle]:
        """
        Get all obstacles within a certain range of a point.
        
        Args:
            x: X coordinate of point
            y: Y coordinate of point
            range_dist: Detection range (meters)
        
        Returns:
            List of obstacles within range
        """
        nearby = []
        for obs in self.obstacles:
            # Distance from point to obstacle surface
            dist = obs.distance_to_point(x, y)
            if dist < range_dist:
                nearby.append(obs)
        return nearby
    
    def check_point_collision(self, x: float, y: float, 
                             safety_margin: float = 0.0) -> bool:
        """
        Check if a point collides with any obstacle.
        
        Args:
            x: X coordinate of point
            y: Y coordinate of point
            safety_margin: Additional safety margin (meters)
        
        Returns:
            True if point collides with any obstacle
        """
        for obs in self.obstacles:
            if obs.is_point_inside(x, y, safety_margin):
                return True
        return False
    
    def check_circle_collision(self, x: float, y: float, radius: float) -> bool:
        """
        Check if a circle collides with any obstacle.
        
        Args:
            x: X coordinate of circle center
            y: Y coordinate of circle center
            radius: Circle radius
        
        Returns:
            True if circle collides with any obstacle
        """
        for obs in self.obstacles:
            if obs.collides_with_circle(x, y, radius):
                return True
        return False
    
    def check_path_collision(self, path_x: np.ndarray, path_y: np.ndarray,
                            robot_radius: float = 0.1) -> Tuple[bool, List[int]]:
        """
        Check if a path collides with any obstacles.
        
        Args:
            path_x: X coordinates of path points
            path_y: Y coordinates of path points
            robot_radius: Radius of robot for collision checking
        
        Returns:
            Tuple of (has_collision, collision_indices)
            - has_collision: True if any point collides
            - collision_indices: List of indices where collisions occur
        """
        collision_indices = []
        
        for i in range(len(path_x)):
            if self.check_circle_collision(path_x[i], path_y[i], robot_radius):
                collision_indices.append(i)
        
        has_collision = len(collision_indices) > 0
        return has_collision, collision_indices
    
    def get_closest_obstacle(self, x: float, y: float) -> Optional[Tuple[Obstacle, float]]:
        """
        Find the closest obstacle to a point.
        
        Args:
            x: X coordinate of point
            y: Y coordinate of point
        
        Returns:
            Tuple of (closest_obstacle, distance) or None if no obstacles
        """
        if not self.obstacles:
            return None
        
        min_dist = float('inf')
        closest = None
        
        for obs in self.obstacles:
            dist = obs.distance_to_point(x, y)
            if dist < min_dist:
                min_dist = dist
                closest = obs
        
        return closest, min_dist
    
    def get_minimum_clearance(self, path_x: np.ndarray, path_y: np.ndarray) -> float:
        """
        Calculate minimum clearance from path to any obstacle.
        
        Args:
            path_x: X coordinates of path points
            path_y: Y coordinates of path points
        
        Returns:
            Minimum distance from path to obstacle surface (meters)
        """
        if not self.obstacles:
            return float('inf')
        
        min_clearance = float('inf')
        
        for i in range(len(path_x)):
            result = self.get_closest_obstacle(path_x[i], path_y[i])
            if result:
                _, dist = result
                min_clearance = min(min_clearance, dist)
        
        return min_clearance
    
    def __len__(self) -> int:
        """Return number of obstacles."""
        return len(self.obstacles)
    
    def __iter__(self):
        """Allow iteration over obstacles."""
        return iter(self.obstacles)
    
    def __repr__(self) -> str:
        return f"ObstacleManager({len(self.obstacles)} obstacles)"
