#!/usr/bin/env python3
"""
Obstacle Avoidance Scenario

Tests obstacle detection and avoidance using hybrid Pure Pursuit + DWA approach.
This demonstrates the bonus feature of obstacle avoidance.
"""

import sys
import os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.robot_simulator import DifferentialDriveRobot
from src.path_smoother import smooth_path
from src.trajectory_generator import generate_trajectory
from src.controller import PurePursuitController
from src.obstacle import ObstacleManager
from src.visualizer import visualize_trajectory_tracking, calculate_metrics, save_metrics, plot_static_results


def main():
    print("=" * 60)
    print("SCENARIO: Obstacle Avoidance (BONUS FEATURE)")
    print("=" * 60)
    
    # Define waypoints that would pass through obstacles
    waypoints = [(0, 0), (1, 0.5), (2, 1), (3, 1.5), (4, 1)]
    print(f"Waypoints: {waypoints}")
    
    # Smooth the path
    print("\n1. Smoothing path...")
    smooth_x, smooth_y = smooth_path(waypoints, sample_spacing=0.08)
    print(f"   Generated {len(smooth_x)} smooth points")
    
    # Generate trajectory
    print("\n2. Generating trajectory...")
    trajectory = generate_trajectory(smooth_x, smooth_y, velocity=0.18)
    print(f"   Trajectory duration: {trajectory[-1][2]:.2f} seconds")
    
    # Create obstacles
    print("\n3. Creating obstacles...")
    obstacle_manager = ObstacleManager()
    
    # Place obstacles along the path - spread out to allow navigation
    obstacles_config = [
        (1.0, 0.2, 0.20),   # Near start, to the side
        (2.2, 1.3, 0.25),   # Middle-upper area
        (3.5, 1.2, 0.20),   # Near end area
    ]
    
    for x, y, r in obstacles_config:
        obstacle_manager.add_circular_obstacle(x, y, r)
    
    print(f"   Created {len(obstacle_manager)} obstacles")
    for i, obs in enumerate(obstacle_manager):
        print(f"      Obstacle {i+1}: center=({obs.x:.2f}, {obs.y:.2f}), radius={obs.radius:.2f}m")
    
    # Check if path would collide without avoidance
    has_collision, collision_indices = obstacle_manager.check_path_collision(
        smooth_x, smooth_y, robot_radius=0.105
    )
    if has_collision:
        print(f"\n   ⚠️  WARNING: Path would collide at {len(collision_indices)} points without avoidance!")
    
    # Create robot and controller
    print("\n4. Creating robot and controller with obstacle avoidance...")
    robot = DifferentialDriveRobot(x=0.0, y=0.0, theta=0.0)
    controller = PurePursuitController(lookahead_distance=0.5, linear_velocity=0.18)
    controller.set_path(smooth_x, smooth_y)
    controller.detection_radius = 1.2  # Increase detection range
    controller.set_obstacles(obstacle_manager)
    
    print(f"   Robot initialized at (0, 0)")
    print(f"   Controller: lookahead={controller.L}m, velocity={controller.v}m/s")
    print(f"   Obstacle avoidance: ENABLED (detection radius={controller.detection_radius}m)")
    
    # Run simulation with visualization
    print("\n5. Running simulation...")
    print("   Close the animation window to continue...")
    print("   NOTE: Robot will avoid obstacles using DWA when nearby")
    
    history_x, history_y, time_history, errors = visualize_trajectory_tracking(
        robot, controller, smooth_x, smooth_y, dt=0.05, max_steps=2000,
        obstacle_manager=obstacle_manager
    )
    
    # Calculate and display metrics
    print("\n6. Performance Metrics:")
    print("-" * 60)
    metrics = calculate_metrics(robot, smooth_x, smooth_y, errors)
    
    # Add obstacle-specific metrics
    min_clearance = obstacle_manager.get_minimum_clearance(
        np.array(robot.history_x), np.array(robot.history_y)
    )
    metrics['Min Obstacle Clearance'] = min_clearance
    
    # Check for collisions
    collision_occurred, _ = obstacle_manager.check_path_collision(
        np.array(robot.history_x), np.array(robot.history_y), robot_radius=0.105
    )
    metrics['Collision Free'] = 1.0 if not collision_occurred else 0.0
    
    for key, value in metrics.items():
        if key == 'Collision Free':
            status = "✓ YES" if value == 1.0 else "✗ NO"
            print(f"   {key}: {status}")
        else:
            print(f"   {key}: {value:.4f}")
    
    # Save metrics
    save_metrics(metrics, "obstacle_avoidance_metrics.json")
    
    # Create static plots
    print("\n7. Generating result plots...")
    plot_static_results(robot, smooth_x, smooth_y, errors, "obstacle_avoidance_results.png",
                       obstacle_manager=obstacle_manager)
    
    print("\n" + "=" * 60)
    if not collision_occurred:
        print("✓ SCENARIO COMPLETE - NO COLLISIONS!")
    else:
        print("✗ SCENARIO COMPLETE - COLLISION DETECTED!")
    print("=" * 60)
    
    # Summary
    print("\nObstacle Avoidance Summary:")
    print(f"  • Successfully avoided {len(obstacle_manager)} obstacles")
    print(f"  • Minimum clearance: {min_clearance:.3f}m")
    print(f"  • Goal reached: {metrics.get('Goal Error', 0.0) < 0.15}")
    print(f"  • Collision-free: {not collision_occurred}")


if __name__ == "__main__":
    main()
