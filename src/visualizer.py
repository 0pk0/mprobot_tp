"""
Visualization Module

Provides matplotlib-based visualization and animation for trajectory tracking.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, FancyArrow
from typing import Tuple, Optional
import json


def visualize_trajectory_tracking(robot, controller, smooth_x, smooth_y,
                                  dt: float = 0.05, max_steps: int = 1000,
                                  save_animation: bool = False,
                                  filename: str = "trajectory_animation.gif",
                                  obstacle_manager=None) -> Tuple:
    """
    Animate robot following the smoothed trajectory with real-time visualization.
    
    Creates a 2-panel figure showing:
        - Left: Robot following path in 2D space
        - Right: Tracking error over time
    
    Args:
        robot: DifferentialDriveRobot instance
        controller: Pure PursuitController instance
        smooth_x: X coordinates of planned path
        smooth_y: Y coordinates of planned path
        dt: Time step for simulation (seconds)
        max_steps: Maximum number of simulation steps
        save_animation: Whether to save animation as GIF
        filename: Filename for saved animation
        obstacle_manager: Optional ObstacleManager for obstacle visualization
    
    Returns:
        Tuple of (history_x, history_y, time_history, errors)
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    errors = []
    
    def animate(frame):
        # Compute control command
        v, omega = controller.compute_control(robot.x, robot.y, robot.theta)
        
        # Calculate tracking error
        error = controller.get_tracking_error(robot.x, robot.y)
        errors.append(error)
        
        # Update robot state
        robot.update(v, omega, dt)
        
        # Check if goal reached
        if v == 0 and omega == 0:
            return []
        
        # Clear axes
        ax1.clear()
        ax2.clear()
        
        # ===== LEFT PANEL: Path Tracking =====
        ax1.plot(smooth_x, smooth_y, 'b-', linewidth=2, label='Planned Path', alpha=0.7)
        ax1.plot(robot.history_x, robot.history_y, 'r--', linewidth=1.5, 
                label='Actual Path', alpha=0.8)
        
        # Plot start and goal
        ax1.plot(smooth_x[0], smooth_y[0], 'go', markersize=10, label='Start')
        ax1.plot(smooth_x[-1], smooth_y[-1], 'r*', markersize=15, label='Goal')
        
        # Plot robot as circle with heading arrow
        robot_circle = Circle((robot.x, robot.y), 0.08, color='green', alpha=0.6, zorder=5)
        ax1.add_patch(robot_circle)
        
        # Draw heading arrow
        arrow_len = 0.15
        arrow = FancyArrow(robot.x, robot.y,
                          arrow_len * np.cos(robot.theta),
                          arrow_len * np.sin(robot.theta),
                          width=0.05, head_width=0.12, head_length=0.08,
                          fc='darkgreen', ec='darkgreen', zorder=6)
        ax1.add_patch(arrow)
        
        # Set plot limits with padding
        x_min, x_max = min(smooth_x) - 1, max(smooth_x) + 1
        y_min, y_max = min(smooth_y) - 1, max(smooth_y) + 1
        ax1.set_xlim([x_min, x_max])
        ax1.set_ylim([y_min, y_max])
        ax1.set_xlabel('X (meters)', fontsize=11)
        ax1.set_ylabel('Y (meters)', fontsize=11)
        ax1.set_title('Trajectory Tracking', fontsize=13, fontweight='bold')
        ax1.legend(loc='best', fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # Draw obstacles if provided
        if obstacle_manager is not None:
            for obs in obstacle_manager:
                obs_circle = Circle((obs.x, obs.y), obs.radius, 
                                  color='red', alpha=0.4, zorder=3, label='Obstacle' if obs == list(obstacle_manager)[0] else '')
                ax1.add_patch(obs_circle)
            # Draw detection radius around robot
            if hasattr(controller,  'detection_radius'):
                detection_circle = Circle((robot.x, robot.y), controller.detection_radius,
                                        color='orange', alpha=0.1, linestyle='--', fill=False, zorder=2)
                ax1.add_patch(detection_circle)
        
        # Add time and velocity text
        info_text = f'Time: {robot.current_time:.2f}s\nV: {v:.3f} m/s\nω: {omega:.3f} rad/s'
        ax1.text(0.02, 0.98, info_text, transform=ax1.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                fontsize=9)
        
        # ===== RIGHT PANEL: Tracking Error =====
        if len(errors) > 1:
            ax2.plot(robot.time_history[:len(errors)], errors, 'r-', linewidth=2)
            ax2.fill_between(robot.time_history[:len(errors)], 0, errors, alpha=0.3, color='red')
            ax2.set_xlabel('Time (seconds)', fontsize=11)
            ax2.set_ylabel('Tracking Error (meters)', fontsize=11)
            ax2.set_title('Cross-Track Error Over Time', fontsize=13, fontweight='bold')
            ax2.grid(True, alpha=0.3)
            
            # Add statistics
            avg_error = np.mean(errors)
            max_error = np.max(errors)
            stats_text = f'Avg: {avg_error:.4f}m\nMax: {max_error:.4f}m'
            ax2.text(0.98, 0.98, stats_text, transform=ax2.transAxes,
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
                    fontsize=9)
        
        return []
    
    anim = FuncAnimation(fig, animate, frames=max_steps, interval=dt*1000,
                        repeat=False, blit=True)
    
    plt.tight_layout()
    
    if save_animation:
        print(f"Saving animation to {filename}...")
        anim.save(filename, writer='pillow', fps=int(1/dt))
        print(f"Animation saved!")
    
    plt.show()
    
    return robot.history_x, robot.history_y, robot.time_history, errors


def plot_static_results(robot, smooth_x, smooth_y, errors, 
                        save_path: Optional[str] = None, obstacle_manager=None):
    """
    Create static plots showing final results.
    
    Args:
        robot: DifferentialDriveRobot instance (after simulation)
        smooth_x: X coordinates of planned path
        smooth_y: Y coordinates of planned path
        errors: List of tracking errors
        save_path: Optional path to save figure
        obstacle_manager: Optional ObstacleManager for obstacle visualization
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Path comparison
    ax = axes[0, 0]
    ax.plot(smooth_x, smooth_y, 'b-', linewidth=2, label='Planned Path')
    ax.plot(robot.history_x, robot.history_y, 'r--', linewidth=1.5, label='Actual Path')
    ax.plot(smooth_x[0], smooth_y[0], 'go', markersize=10, label='Start')
    ax.plot(smooth_x[-1], smooth_y[-1], 'r*', markersize=15, label='Goal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Path Tracking Results')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Draw obstacles if provided
    if obstacle_manager is not None:
        for obs in obstacle_manager:
            obs_circle = Circle((obs.x, obs.y), obs.radius, 
                              color='red', alpha=0.4, zorder=3)
            ax.add_patch(obs_circle)
        # Add obstacle legend
        ax.plot([], [], 'o', color='red', alpha=0.4, markersize=10, label='Obstacles')
        ax.legend()
    
    # Plot 2: Tracking error over time
    ax = axes[0, 1]
    if len(errors) > 0:
        ax.plot(robot.time_history[:len(errors)], errors, 'r-', linewidth=2)
        ax.fill_between(robot.time_history[:len(errors)], 0, errors, alpha=0.3, color='red')
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Tracking Error (meters)')
        ax.set_title('Cross-Track Error')
        ax.grid(True, alpha=0.3)
    
    # Plot 3: Heading angle over time
    ax = axes[1, 0]
    ax.plot(robot.time_history, np.rad2deg(robot.history_theta), 'g-', linewidth=2)
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Heading Angle (degrees)')
    ax.set_title('Robot Orientation Over Time')
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Velocity profile
    ax = axes[1, 1]
    if len(robot.history_x) > 1:
        velocities = []
        for i in range(1, len(robot.history_x)):
            dx = robot.history_x[i] - robot.history_x[i-1]
            dy = robot.history_y[i] - robot.history_y[i-1]
            dt = robot.time_history[i] - robot.time_history[i-1]
            if dt > 0:
                v = np.sqrt(dx**2 + dy**2) / dt
                velocities.append(v)
        
        ax.plot(robot.time_history[1:len(velocities)+1], velocities, 'b-', linewidth=2)
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
    
    plt.show()


def calculate_metrics(robot, smooth_x, smooth_y, errors) -> dict:
    """
    Calculate performance metrics for trajectory tracking.
    
    Args:
        robot: DifferentialDriveRobot instance
        smooth_x: Planned path X coordinates
        smooth_y: Planned path Y coordinates
        errors: List of tracking errors
    
    Returns:
        Dictionary of metrics
    """
    metrics = {}
    
    if len(errors) > 0:
        metrics['avg_error'] = float(np.mean(errors))
        metrics['max_error'] = float(np.max(errors))
        metrics['min_error'] = float(np.min(errors))
        metrics['std_error'] = float(np.std(errors))
    
    # Goal reaching error
    final_x, final_y = robot.history_x[-1], robot.history_y[-1]
    goal_x, goal_y = smooth_x[-1], smooth_y[-1]
    metrics['goal_error'] = float(np.sqrt((final_x - goal_x)**2 + (final_y - goal_y)**2))
    
    # Completion time
    metrics['completion_time'] = float(robot.current_time)
    
    # Path length
    actual_length = 0
    for i in range(1, len(robot.history_x)):
        dx = robot.history_x[i] - robot.history_x[i-1]
        dy = robot.history_y[i] - robot.history_y[i-1]
        actual_length += np.sqrt(dx**2 + dy**2)
    metrics['actual_path_length'] = float(actual_length)
    
    planned_length = 0
    for i in range(1, len(smooth_x)):
        dx = smooth_x[i] - smooth_x[i-1]
        dy = smooth_y[i] - smooth_y[i-1]
        planned_length += np.sqrt(dx**2 + dy**2)
    metrics['planned_path_length'] = float(planned_length)
    
    return metrics


def save_metrics(metrics: dict, filename: str = "metrics.json"):
    """Save metrics to JSON file."""
    with open(filename, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"Metrics saved to {filename}")
