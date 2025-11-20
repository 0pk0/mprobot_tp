"""
Trajectory Generation Module

Converts smoothed spatial paths into time-parameterized trajectories
with velocity profiles.
"""

import numpy as np
from typing import List, Tuple


def generate_trajectory(smooth_x: np.ndarray, smooth_y: np.ndarray, 
                       velocity: float = 0.2) -> List[Tuple[float, float, float, float]]:
    """
    Add time stamps to smoothed path based on constant velocity profile.
    
    This function creates a time-parameterized trajectory by calculating
    the time required to traverse each segment at the specified constant velocity.
    
    Args:
        smooth_x: X coordinates of smoothed path
        smooth_y: Y coordinates of smoothed path
        velocity: Desired constant velocity in m/s (default: 0.2)
    
    Returns:
        List of (x, y, t, v) tuples where:
            x, y: Position coordinates (meters)
            t: Time stamp (seconds)
            v: Velocity at that point (m/s)
    
    Raises:
        ValueError: If velocity is non-positive or paths are empty
    """
    if velocity <= 0:
        raise ValueError("Velocity must be positive")
    
    if len(smooth_x) == 0 or len(smooth_y) == 0:
        raise ValueError("Empty path provided")
    
    if len(smooth_x) != len(smooth_y):
        raise ValueError("X and Y coordinates must have same length")
    
    trajectory = []
    t = 0.0
    
    # Add first point at t=0
    trajectory.append((float(smooth_x[0]), float(smooth_y[0]), t, velocity))
    
    # Calculate time stamps for remaining points
    for i in range(1, len(smooth_x)):
        # Calculate distance to next point
        dist = np.sqrt((smooth_x[i] - smooth_x[i-1])**2 + 
                      (smooth_y[i] - smooth_y[i-1])**2)
        
        # Calculate time increment: dt = distance / velocity
        dt = dist / velocity
        t += dt
        
        trajectory.append((float(smooth_x[i]), float(smooth_y[i]), t, velocity))
    
    return trajectory


def generate_trajectory_with_trapezoidal_profile(
    smooth_x: np.ndarray, 
    smooth_y: np.ndarray,
    max_velocity: float = 0.2,
    max_acceleration: float = 0.5,
    max_deceleration: float = 0.5
) -> List[Tuple[float, float, float, float]]:
    """
    Generate trajectory with trapezoidal velocity profile (acceleration/cruise/deceleration).
    
    This is a more advanced velocity profile that includes acceleration and
    deceleration phases for smoother motion.
    
    Args:
        smooth_x: X coordinates of smoothed path
        smooth_y: Y coordinates of smoothed path
        max_velocity: Maximum velocity in m/s
        max_acceleration: Maximum acceleration in m/s²
        max_deceleration: Maximum deceleration in m/s²
    
    Returns:
        List of (x, y, t, v) tuples with time-varying velocities
    """
    if len(smooth_x) < 2:
        return [(float(smooth_x[0]), float(smooth_y[0]), 0.0, 0.0)]
    
    # Calculate cumulative distances along path
    distances = [0.0]
    for i in range(1, len(smooth_x)):
        dist = np.sqrt((smooth_x[i] - smooth_x[i-1])**2 + 
                      (smooth_y[i] - smooth_y[i-1])**2)
        distances.append(distances[-1] + dist)
    
    total_distance = distances[-1]
    
    # Calculate phases of trapezoidal profile
    # Acceleration distance: v² = 2*a*d => d = v²/(2*a)
    accel_dist = max_velocity**2 / (2 * max_acceleration)
    decel_dist = max_velocity**2 / (2 * max_deceleration)
    
    # Check if we have enough distance for full trapezoid
    if accel_dist + decel_dist > total_distance:
        # Triangular profile instead (no cruise phase)
        peak_velocity = np.sqrt(max_acceleration * max_deceleration * total_distance / 
                               (max_acceleration + max_deceleration))
        accel_dist = peak_velocity**2 / (2 * max_acceleration)
        decel_dist = peak_velocity**2 / (2 * max_deceleration)
        cruise_dist = 0
    else:
        peak_velocity = max_velocity
        cruise_dist = total_distance - accel_dist - decel_dist
    
    trajectory = []
    t = 0.0
    current_velocity = 0.0
    
    for i, dist in enumerate(distances):
        # Determine which phase we're in and calculate velocity
        if dist <= accel_dist:
            # Acceleration phase: v = sqrt(2*a*d)
            current_velocity = np.sqrt(2 * max_acceleration * dist)
        elif dist <= accel_dist + cruise_dist:
            # Cruise phase: constant velocity
            current_velocity = peak_velocity
        else:
            # Deceleration phase
            remaining_dist = total_distance - dist
            current_velocity = np.sqrt(2 * max_deceleration * remaining_dist)
        
        # Calculate time increment
        if i > 0:
            segment_dist = distances[i] - distances[i-1]
            avg_velocity = (current_velocity + trajectory[-1][3]) / 2
            if avg_velocity > 0:
                dt = segment_dist / avg_velocity
                t += dt
        
        trajectory.append((float(smooth_x[i]), float(smooth_y[i]), t, current_velocity))
    
    return trajectory
