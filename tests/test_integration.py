"""
Integration tests for the complete trajectory tracking pipeline.
"""

import pytest
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.robot_simulator import DifferentialDriveRobot
from src.path_smoother import smooth_path
from src.trajectory_generator import generate_trajectory
from src.controller import PurePursuitController


class TestIntegration:
    """Integration tests for full pipeline."""
    
    def test_complete_pipeline_straight_line(self):
        """Test complete pipeline on straight line."""
        # Define waypoints
        waypoints = [(0, 0), (2, 0), (4, 0)]
        
        # Smooth path
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        assert len(smooth_x) > len(waypoints)
        
        # Generate trajectory
        trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.2)
        assert len(trajectory) == len(smooth_x)
        assert trajectory[0][2] == 0.0  # First time stamp is 0
        assert trajectory[-1][2] > 0.0  # Last time stamp is positive
        
        # Create robot and controller
        robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
        controller = PurePursuitController(lookahead_distance=0.5, linear_velocity=0.2)
        controller.set_path(smooth_x, smooth_y)
        
        # Run simulation for a few steps
        for _ in range(100):
            v, omega = controller.compute_control(robot.x, robot.y, robot.theta)
            robot.update(v, omega, dt=0.05)
            
            if v == 0 and omega == 0:
                break  # Goal reached
        
        # Robot should have moved forward
        assert robot.x > 1.0
        assert abs(robot.y) < 0.5  # Should stay close to y=0
    
    def test_complete_pipeline_curved_path(self):
        """Test complete pipeline on curved path."""
        waypoints = [(0, 0), (1, 1), (2, 0)]
        
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.08)
        trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.2)
        
        robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=np.radians(45))
        controller = PurePursuitController(lookahead_distance=0.4, linear_velocity=0.2)
        controller.set_path(smooth_x, smooth_y)
        
        errors = []
        for _ in range(200):
            v, omega = controller.compute_control(robot.x, robot.y, robot.theta)
            error = controller.get_tracking_error(robot.x, robot.y)
            errors.append(error)
            robot.update(v, omega, dt=0.05)
            
            if v == 0 and omega == 0:
                break
        
        # Check that average tracking error is reasonable
        avg_error = np.mean(errors)
        assert avg_error < 0.2, f"Average tracking error too high: {avg_error}"
    
    def test_goal_reaching(self):
        """Test that robot reaches the goal."""
        waypoints = [(0, 0), (1, 0), (2, 0), (3, 0)]
        
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        
        robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
        controller = PurePursuitController()
        controller.set_path(smooth_x, smooth_y)
        
        max_steps = 500
        for step in range(max_steps):
            v, omega = controller.compute_control(robot.x, robot.y, robot.theta)
            robot.update(v, omega, dt=0.05)
            
            if v == 0 and omega == 0:
                break
        
        # Check that goal was reached
        goal_x, goal_y = smooth_x[-1], smooth_y[-1]
        final_dist = np.sqrt((robot.x - goal_x)**2 + (robot.y - goal_y)**2)
        assert final_dist < controller.goal_tolerance, \
            f"Robot didn't reach goal. Distance: {final_dist}"
    
    def test_trajectory_time_monotonic(self):
        """Test that trajectory timestamps are monotonically increasing."""
        waypoints = [(0, 0), (1, 1), (2, 1), (3, 0)]
        smooth_x, smooth_y = smooth_path(waypoints)
        trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.2)
        
        times = [t[2] for t in trajectory]
        
        # Check monotonic increase
        for i in range(1, len(times)):
            assert times[i] >= times[i-1], "Times not monotonically increasing"
    
    def test_robot_history_tracking(self):
        """Test that robot properly tracks history."""
        robot = DifferentialDriveRobot(x=0, y=0, theta=0)
        
        initial_history_length = len(robot.history_x)
        
        # Move robot a few steps
        for _ in range(10):
                robot.update(v=0.2, omega=0.1, dt=0.05)
        
        # History should have grown
        assert len(robot.history_x) == initial_history_length + 10
        assert len(robot.history_y) == initial_history_length + 10
        assert len(robot.history_theta) == initial_history_length + 10
        assert len(robot.time_history) == initial_history_length + 10
    
    def test_robot_reset(self):
        """Test robot reset functionality."""
        robot = DifferentialDriveRobot(x=0, y=0, theta=0)
        
        # Move robot
        for _ in range(5):
            robot.update(v=0.2, omega=0.0, dt=0.05)
        
        # Reset
        robot.reset(x=1, y=1, theta=np.pi/2)
        
        assert robot.x == 1
        assert robot.y == 1
        assert robot.theta == np.pi/2
        assert len(robot.history_x) == 1
        assert robot.current_time == 0.0
    
    def test_obstacle_avoidance_integration(self):
        """Test complete pipeline with obstacles."""
        from src.obstacle import ObstacleManager
        
        # Define waypoints
        waypoints = [(0, 0), (2, 0), (4, 0)]
        
        # Smooth path
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        
        # Create obstacles
        obstacle_manager = ObstacleManager()
        obstacle_manager.add_circular_obstacle(2, 0, 0.3)  # Obstacle in path
        
        # Create robot and controller
        robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
        controller = PurePursuitController(lookahead_distance=0.4, linear_velocity=0.18)
        controller.set_path(smooth_x, smooth_y)
        controller.set_obstacles(obstacle_manager)
        
        # Run simulation
        for _ in range(300):
            v, omega = controller.compute_control(robot.x, robot.y, robot.theta)
            robot.update(v, omega, dt=0.05)
            
            if v == 0 and omega == 0:
                break
        
        # Check that robot avoided obstacle
        collision, _ = obstacle_manager.check_path_collision(
            np.array(robot.history_x), np.array(robot.history_y), robot_radius=0.105
        )
        assert not collision, "Robot collided with obstacle"
        
        # Check that robot still made progress
        assert robot.x > 1.5  # Should have moved significantly
    
    def test_obstacle_detection(self):
        """Test obstacle detection in controller."""
        from src.obstacle import ObstacleManager
        
        controller = PurePursuitController()
        waypoints = [(0, 0), (5, 0)]
        smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.1)
        controller.set_path(smooth_x, smooth_y)
        
        # No obstacles initially
        assert not controller.has_nearby_obstacles(0, 0)
        
        # Add obstacles
        obstacle_manager = ObstacleManager()
        obstacle_manager.add_circular_obstacle(0.5, 0, 0.2)  # Close obstacle
        obstacle_manager.add_circular_obstacle(5, 0, 0.2)  # Far obstacle
        controller.set_obstacles(obstacle_manager)
        
        # Should detect close obstacle
        assert controller.has_nearby_obstacles(0, 0)
        
        # Should not detect obstacle when far
        assert not controller.has_nearby_obstacles(10, 0)

