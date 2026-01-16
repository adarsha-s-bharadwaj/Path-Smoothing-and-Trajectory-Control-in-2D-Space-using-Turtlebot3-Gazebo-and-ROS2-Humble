"""
trajectory_generator.py

This module is responsible for generating a smooth, time-parameterized
trajectory from discrete waypoints stored in a YAML file.

Key features:
- Loads waypoints from ROS 2 package config
- Uses cubic B-spline interpolation for geometric smoothing
- Assigns timestamps assuming constant linear velocity

Output:
- A list of (x, y, t) tuples where t is time from trajectory start
"""

import yaml
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev

def generate_trajectory(constant_velocity=0.25, ds=0.05):
    """
    Generate a smooth, time-parameterized trajectory.

    Parameters
    ----------
    constant_velocity : float
        Desired linear velocity of the robot (m/s).
    ds : float
        Approximate spatial resolution used to sample the spline.

    Returns
    -------
    trajectory : list of tuples
        List of (x, y, t) where:
        - x, y : position in meters
        - t    : time from trajectory start in seconds
    """
    
    pkg_path = get_package_share_directory('nav_assignment')
    yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    waypoints = np.array(data['waypoints'])
    x, y = waypoints[:, 0], waypoints[:, 1]

    # Spline smoothing (geometry only)
    tck, _ = splprep([x, y], s=0.0)
    u = np.linspace(0, 1, int(len(x) / ds * 5))
    xs, ys = splev(u, tck)
    
    # ----------------------------------------------------
    # Time parameterization (constant velocity assumption)
    # ----------------------------------------------------
    trajectory = []
    time = 0.0

    for i in range(len(xs)):
        if i > 0:
            # Euclidean distance between consecutive points
            dist = np.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
            
            # Time increment = distance / velocity
            time += dist / constant_velocity

        trajectory.append((xs[i], ys[i], time))

    return trajectory
