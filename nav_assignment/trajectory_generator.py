import yaml
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev

def generate_trajectory(constant_velocity=0.25, ds=0.05):
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

    trajectory = []
    time = 0.0

    for i in range(len(xs)):
        if i > 0:
            dist = np.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
            time += dist / constant_velocity

        trajectory.append((xs[i], ys[i], time))

    return trajectory
