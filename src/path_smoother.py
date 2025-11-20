"""
Path Smoothing Module

Transforms discrete waypoints into smooth continuous paths using
cubic spline interpolation with arc-length parameterization.
"""

import numpy as np
from scipy.interpolate import CubicSpline
from typing import List, Tuple


def smooth_path(waypoints: List[Tuple[float, float]], 
                sample_spacing: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Smooth discrete waypoints using cubic spline interpolation.
    
    This function uses arc-length parameterization to create a natural
    cubic spline through the given waypoints, then samples it at uniform
    intervals to create a smooth path.
    
    Args:
        waypoints: List of (x, y) tuples representing discrete waypoints
        sample_spacing: Distance between sampled points in meters (default: 0.1m)
    
    Returns:
        Tuple of (smooth_x, smooth_y) as numpy arrays of smoothed coordinates
    
    Raises:
        ValueError: If fewer than 2 waypoints are provided
    """
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints to smooth a path")
    
    # Extract x and y coordinates
    x = np.array([wp[0] for wp in waypoints])
    y = np.array([wp[1] for wp in waypoints])
    
    # Calculate arc length parameterization
    # This ensures uniform spacing along the curve
    distances = [0.0]
    for i in range(1, len(x)):
        dist = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
        distances.append(distances[-1] + dist)
    
    distances = np.array(distances)
    
    # Handle edge case: all waypoints are the same
    if distances[-1] < 1e-6:
        return x, y
    
    # Create cubic splines for x and y as functions of arc length
    cs_x = CubicSpline(distances, x)
    cs_y = CubicSpline(distances, y)
    
    # Sample the splines at uniform intervals along the arc length
    # Add small epsilon to avoid floating point issues at the end
    num_samples = int(distances[-1] / sample_spacing) + 1
    s_new = np.linspace(0, distances[-1], num_samples)
    
    smooth_x = cs_x(s_new)
    smooth_y = cs_y(s_new)
    
    return smooth_x, smooth_y


def calculate_curvature(smooth_x: np.ndarray, smooth_y: np.ndarray) -> np.ndarray:
    """
    Calculate curvature along the smoothed path.
    
    Curvature κ is calculated using finite differences:
    κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    
    Args:
        smooth_x: X coordinates of smooth path
        smooth_y: Y coordinates of smooth path
    
    Returns:
        Array of curvature values at each point
    """
    # First derivatives
    dx = np.gradient(smooth_x)
    dy = np.gradient(smooth_y)
    
    # Second derivatives
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature formula
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx**2 + dy**2)**(3/2)
    
    # Avoid division by zero
    denominator[denominator < 1e-10] = 1e-10
    
    curvature = numerator / denominator
    
    return curvature
