#!/usr/bin/env python3
"""
Curved Path Scenario

Tests path smoothing and trajectory tracking on a curved path.
This tests the controller's ability to handle smooth curves.
"""

import sys
import os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.robot_simulator import DifferentialDriveRobot
from src.path_smoother import smooth_path
from src.trajectory_generator import generate_trajectory
from src.controller import PurePursuitController
from src.visualizer import visualize_trajectory_tracking, calculate_metrics, save_metrics, plot_static_results


def main():
    print("=" * 60)
    print("SCENARIO: Curved Path")
    print("=" * 60)
    
    # Define waypoints for curved path
    waypoints = [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1)]
    print(f"Waypoints: {waypoints}")
    
    # Smooth the path
    print("\n1. Smoothing path...")
    smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.08)
    print(f"   Generated {len(smooth_x)} smooth points")
    
    # Generate trajectory
    print("\n2. Generating trajectory...")
    trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.2)
    print(f"   Trajectory duration: {trajectory[-1][2]:.2f} seconds")
    
    # Create robot and controller
    print("\n3. Creating robot and controller...")
    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=np.radians(45))  # Start angled
    controller = PurePursuitController(lookahead_distance=0.4, linear_velocity=0.2)
    controller.set_path(smooth_x, smooth_y)
    print("   Robot initialized at (0, 0) with 45° heading")
    print(f"   Controller: lookahead={controller.L}m, velocity={controller.v}m/s")
    
    # Run simulation with visualization
    print("\n4. Running simulation...")
    print("   Close the animation window to continue...")
    history_x, history_y, time_history, errors = visualize_trajectory_tracking(
        robot, controller, smooth_x, smooth_y, dt=0.05, max_steps=1500
    )
    
    # Calculate and display metrics
    print("\n5. Performance Metrics:")
    print("-" * 60)
    metrics = calculate_metrics(robot, smooth_x, smooth_y, errors)
    for key, value in metrics.items():
        print(f"   {key}: {value:.4f}")
    
    # Save metrics
    save_metrics(metrics, "curved_path_metrics.json")
    
    # Create static plots
    print("\n6. Generating result plots...")
    plot_static_results(robot, smooth_x, smooth_y, errors, "curved_path_results.png")
    
    print("\n" + "=" * 60)
    print("SCENARIO COMPLETE!")
    print("=" * 60)


if __name__ == "__main__":
    main()
