"""
Can the robot be sent there? A quick look at the map before Nav2 gets a goal (no ROS).

Goals on a wall, in unmapped space, off the map or closer to a wall than the
robot's half-width are rejected up front with the reason, instead of Nav2
failing after minutes of replanning and recoveries.
"""
import math

import numpy as np

OCCUPIED = 65   # map_server's occupied threshold (occupancy 0..100)


def goal_problem(resolution, width, height, origin_x, origin_y, data, x, y, clearance):
    """Return why (x, y) is not a valid goal on this occupancy grid, or None if it is."""
    i = math.floor((x - origin_x) / resolution)
    j = math.floor((y - origin_y) / resolution)
    if not (0 <= i < width and 0 <= j < height):
        return 'xəritədən kənardır'
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    value = grid[j, i]
    if value < 0:
        return 'xəritələnməmiş (naməlum) zonadır'
    if value >= OCCUPIED:
        return 'divarın / maneənin üstündədir'
    r = int(math.ceil(clearance / resolution))
    j0, j1, i0, i1 = max(0, j - r), min(height, j + r + 1), max(0, i - r), min(width, i + r + 1)
    jj, ii = np.nonzero(grid[j0:j1, i0:i1] >= OCCUPIED)
    if len(jj):
        dist = np.hypot((jj + j0 - j) * resolution, (ii + i0 - i) * resolution).min()
        if dist < clearance:
            return (f'divara çox yaxındır ({dist * 100:.0f} sm; robotun mərkəzinə '
                    f'{clearance * 100:.0f} sm boş yer lazımdır)')
    return None
