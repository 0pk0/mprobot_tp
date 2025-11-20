#!/usr/bin/env python3
"""
Sharp Turn Scenario

Tests path smoothing and trajectory tracking on a path with sharp 90-degree turn.
This is the most challenging scenario, testing the controller's handling of sharp corners.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.robot_simulator import DifferentialDriveRobot
from src.path_smoother import smooth_path
from src.trajectory_generator import generate_trajectory
from src.controller import PurePursuitController
from src.visualizer import visualize_trajectory_tracking, calculate_metrics, save_metrics, plot_static_results


def main():
    print("=" * 60)
    print("SCENARIO: Sharp Turn (90-degree)")
    print("=" * 60)
    
    # Define waypoints for sharp turn
    waypoints = [(0, 0), (2, 0), (2, 2), (4, 2)]
    print(f"Waypoints: {waypoints}")
    
    # Smooth the path
    print("\n1. Smoothing path...")
    smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.05)
    print(f"   Generated {len(smooth_x)} smooth points")
    
    # Generate trajectory
    print("\n2. Generating trajectory...")
    trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.15)  # Slower for sharp turn
    print(f"   Trajectory duration: {trajectory[-1][2]:.2f} seconds")
    
    # Create robot and controller
    print("\n3. Creating robot and controller...")
    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
    controller = PurePursuitController(lookahead_distance=0.3, linear_velocity=0.15)  # Shorter lookahead
    controller.set_path(smooth_x, smooth_y)
    print("   Robot initialized at (0, 0)")
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
    save_metrics(metrics, "sharp_turn_metrics.json")
    
    # Create static plots
    print("\n6. Generating result plots...")
    plot_static_results(robot, smooth_x, smooth_y, errors, "sharp_turn_results.png")
    
    print("\n" + "=" * 60)
    print("SCENARIO COMPLETE!")
    print("=" * 60)


if __name__ == "__main__":
    main()
