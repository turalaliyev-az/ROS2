"""
Where is the robot on the map? Correlative scan matching (no ROS).

The lidar scan (points in the robot frame) is tried at candidate poses; each
pose scores how close the points land to mapped walls (a likelihood field).
A coarse search over positions and headings is followed by a fine one around
the best distinct candidates. The answer is trusted only when the scan fits
well and no other place fits nearly as well (look-alike rooms).
"""
import math

import numpy as np
from scipy import ndimage

OCCUPIED = 65
SIGMA = 0.08        # m: how far from a wall a scan point may land and still count
HIT_DIST = 0.10     # m: a point this close to a wall is a hit
MIN_HITS = 0.6      # fraction of scan points on walls for a trusted pose
MIN_MARGIN = 0.08   # score lead over the best other place
TIE = 0.02          # scores this close are look-alike places


class MapField:
    """A map prepared for matching: distance to walls and likelihood per cell."""

    def __init__(self, resolution, width, height, origin_x, origin_y, data):
        self.res, self.ox, self.oy = resolution, origin_x, origin_y
        self.grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.dist = ndimage.distance_transform_edt(~(self.grid >= OCCUPIED)) * resolution
        self.like = np.exp(-0.5 * (np.minimum(self.dist, 1.0) / SIGMA) ** 2).astype(np.float32)

    def score(self, xy, theta, pts):
        """Mean likelihood of scan `pts` (Nx2) at robot positions `xy` (Mx2) with heading theta."""
        c, s = math.cos(theta), math.sin(theta)
        wx = xy[:, None, 0] + c * pts[None, :, 0] - s * pts[None, :, 1]
        wy = xy[:, None, 1] + s * pts[None, :, 0] + c * pts[None, :, 1]
        i = np.floor((wx - self.ox) / self.res).astype(np.int32)
        j = np.floor((wy - self.oy) / self.res).astype(np.int32)
        h, w = self.grid.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        v = np.zeros(i.shape, dtype=np.float32)
        v[inside] = self.like[j[inside], i[inside]]
        return v.mean(axis=1)

    def hit_ratio(self, x, y, theta, pts):
        """Get the fraction of scan points within HIT_DIST of a mapped wall at this pose."""
        c, s = math.cos(theta), math.sin(theta)
        i = np.floor((x + c * pts[:, 0] - s * pts[:, 1] - self.ox) / self.res).astype(int)
        j = np.floor((y + s * pts[:, 0] + c * pts[:, 1] - self.oy) / self.res).astype(int)
        h, w = self.grid.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        if not inside.any():
            return 0.0
        return float(np.mean(self.dist[j[inside], i[inside]] < HIT_DIST))

    def candidates(self, clearance, lattice, center=None, radius=None):
        """Free positions with room for the robot, on a lattice (within radius of center)."""
        free = (self.grid >= 0) & (self.grid < 25) & (self.dist >= clearance)
        step = max(1, int(round(lattice / self.res)))
        jj, ii = np.nonzero(free[::step, ::step])
        xy = np.stack([self.ox + (ii * step + 0.5) * self.res,
                       self.oy + (jj * step + 0.5) * self.res], axis=1)
        if center is not None:
            xy = xy[np.hypot(xy[:, 0] - center[0], xy[:, 1] - center[1]) <= radius]
        return xy


def thin(pts, bearing_step_deg=1.5):
    """Keep one point per bearing step (scans merged from several sweeps repeat points)."""
    if len(pts) == 0:
        return pts
    bearing = np.round(np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) / bearing_step_deg).astype(int)
    _, first = np.unique(bearing, return_index=True)
    return pts[first]


def localize(field, pts, clearance=0.25, center=None, radius=None, yaw=None, yaw_window=None,
             lattice=0.2, angle_step_deg=4.0):
    """
    Search for the robot's pose; return a dict (x, y, yaw, score, hits, margin, trusted).

    center/radius limit the positions, yaw/yaw_window the headings (a local
    search around a known pose); without them the whole map is searched.
    """
    xy = field.candidates(clearance, lattice, center, radius)
    if len(xy) == 0:
        return None
    if yaw is None:
        headings = np.radians(np.arange(0.0, 360.0, angle_step_deg))
    else:
        headings = yaw + np.radians(np.arange(-yaw_window, yaw_window + 1e-9, angle_step_deg))
    coarse = []
    for th in headings:
        sc = np.concatenate([field.score(xy[n:n + 2000], th, pts)
                             for n in range(0, len(xy), 2000)])
        top = np.argsort(sc)[-30:]
        coarse += [(float(sc[k]), xy[k, 0], xy[k, 1], float(th)) for k in top]
    coarse.sort(reverse=True)
    distinct = []
    for c in coarse:
        if all(math.hypot(c[1] - d[1], c[2] - d[2]) > 1.0 or
               abs(math.remainder(c[3] - d[3], 2 * math.pi)) > math.radians(30) for d in distinct):
            distinct.append(c)
        if len(distinct) >= 8:
            break
    refined = sorted((_refine(field, pts, c[1], c[2], c[3]) for c in distinct), reverse=True)
    sc, x, y, th = refined[0]
    other = next((r for r in refined[1:] if math.hypot(r[1] - x, r[2] - y) > 1.0 or
                  abs(math.remainder(r[3] - th, 2 * math.pi)) > math.radians(30)), None)
    hits = field.hit_ratio(x, y, th, pts)
    margin = sc - (other[0] if other else 0.0)
    return {'x': x, 'y': y, 'yaw': math.remainder(th, 2 * math.pi), 'score': sc, 'hits': hits,
            'margin': margin, 'trusted': hits >= MIN_HITS and margin >= MIN_MARGIN}


def localize_near(field, pts, hint, clearance=0.25):
    """
    Search the whole map; a hint (x, y, yaw) only settles look-alike places.

    A search around the hint alone is not enough: a fair fit near a stale
    hint may hide a far better one where the robot really is. So the map
    search decides, and when it is ambiguous, an unambiguous fit near the
    hint that scores as well as the best is taken. Adds 'by_hint'.
    """
    best = localize(field, pts, clearance)
    if best is not None:
        best['by_hint'] = False
    if best is None or best['trusted'] or hint is None:
        return best
    near = localize(field, pts, clearance, center=hint[:2], radius=1.5,
                    yaw=hint[2], yaw_window=45)
    if near and near['trusted'] and near['score'] >= best['score'] - TIE:
        near['by_hint'] = True
        return near
    return best


def carry(pose, odom_then, odom_now):
    """
    Move a map pose (x, y, yaw) by what odometry saw since.

    The robot was at `pose` when odometry read `odom_then`; where is it at `odom_now`?
    """
    x, y, th = pose
    dx, dy = odom_now[0] - odom_then[0], odom_now[1] - odom_then[1]
    c, s = math.cos(odom_then[2]), math.sin(odom_then[2])
    fx, fy = c * dx + s * dy, -s * dx + c * dy      # motion in the robot frame back then
    return (x + math.cos(th) * fx - math.sin(th) * fy,
            y + math.sin(th) * fx + math.cos(th) * fy,
            math.remainder(th + odom_now[2] - odom_then[2], 2 * math.pi))


def _refine(field, pts, x, y, th):
    for span, dstep, aspan, astep in ((0.3, 0.05, 6.0, 1.0), (0.06, 0.02, 1.5, 0.5)):
        offs = np.arange(-span, span + 1e-9, dstep)
        gx, gy = np.meshgrid(offs + x, offs + y)
        xy = np.stack([gx.ravel(), gy.ravel()], axis=1)
        best = (-1.0, x, y, th)
        for da in np.arange(-aspan, aspan + 1e-9, astep):
            sc = field.score(xy, th + math.radians(da), pts)
            k = int(np.argmax(sc))
            if sc[k] > best[0]:
                best = (float(sc[k]), xy[k, 0], xy[k, 1], th + math.radians(da))
        _, x, y, th = best
    return best
