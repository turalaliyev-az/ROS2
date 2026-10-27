"""
Where are the things the detector boxed? Pure numpy, no ROS.

The detector boxes objects in the colour image; the depth image (a different
sensor, field of view and resolution) says how far away things are. Depth
pixels are turned into 3D points, moved into the colour camera's frame and
projected onto the colour image, so every box can be given the 3D points that
fall inside it.
"""
import math

import numpy as np


def depth_points(depth_m, K, stride=2):
    """Turn a depth image in metres (0 = no reading) into Nx3 depth optical frame points."""
    d = depth_m[::stride, ::stride]
    rows, cols = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    z = d.ravel()
    ok = z > 0
    u = cols.ravel()[ok] * stride
    v = rows.ravel()[ok] * stride
    z = z[ok]
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    return np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1)


def scale_intrinsics(K, from_size, to_size):
    """
    Adapt camera matrix K (row-major, 9 values) to an image resized to `to_size`.

    The RealSense decimation filter shrinks the depth image; if its camera_info
    still describes the full-size image, the points would land in the wrong
    place. Sizes are (width, height); an unknown `from_size` leaves K as is.
    """
    k = [float(v) for v in K]
    (fw, fh), (tw, th) = from_size, to_size
    if not fw or not fh or (fw, fh) == (tw, th):
        return k
    sx, sy = tw / fw, th / fh
    k[0] *= sx
    k[2] = (k[2] + 0.5) * sx - 0.5
    k[4] *= sy
    k[5] = (k[5] + 0.5) * sy - 0.5
    return k


def transform(points, rotation, translation):
    """Move Nx3 points by a rigid motion: 3x3 rotation, then 3-vector translation."""
    return points @ np.asarray(rotation).T + np.asarray(translation)


def project(points, K):
    """Project Nx3 optical frame points to pixels; also a mask of points in front of the camera."""
    z = points[:, 2]
    front = z > 0.05
    safe = np.where(front, z, 1.0)
    px = np.stack([K[0] * points[:, 0] / safe + K[2], K[4] * points[:, 1] / safe + K[5]], axis=1)
    return px, front


def locate(points, pixels, box, shrink=0.6, depth_band=0.4, min_points=6):
    """
    Find the object in `box`: (centre, surface, near), or None.

    Only the inner part of the box (`shrink`) and the nearest depth band are
    used: around a person or between chair legs the box also contains the
    wall behind, which must not pull the estimate away.
    centre:  median of that band, roughly the middle of what is visible.
    surface: the point at depth `near` on the same line of sight, i.e. the
             object's near side, which a robot driving up to it meets first.
    near:    depth (along the optical axis) of the near side.
    """
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    hw, hh = (x2 - x1) * shrink / 2.0, (y2 - y1) * shrink / 2.0
    inside = (np.abs(pixels[:, 0] - cx) <= hw) & (np.abs(pixels[:, 1] - cy) <= hh)
    if inside.sum() < min_points:
        return None
    pts = points[inside]
    z = pts[:, 2]
    near = float(np.percentile(z, 20))
    band = pts[(z >= near - 0.05) & (z <= near + depth_band)]
    if len(band) < min_points // 2:
        return None
    centre = np.median(band, axis=0)
    surface = centre * (near / centre[2]) if centre[2] > 0 else centre
    return centre, surface, near


def person_rings(people, radius, count=12, height=0.5, exclude=None, exclusion_radius=0.0):
    """
    Turn each person (x, y) into `count` points on a circle of `radius`, at `height`.

    People closer than `exclusion_radius` to `exclude` (x, y) are left out.
    Returns an Nx3 float32 array (N may be 0).
    """
    angles = 2.0 * math.pi * np.arange(count) / count
    rings = []
    for x, y in people:
        if exclude is not None and math.hypot(x - exclude[0], y - exclude[1]) < exclusion_radius:
            continue
        rings.append(np.stack([x + radius * np.cos(angles), y + radius * np.sin(angles),
                               np.full(count, height)], axis=1))
    if not rings:
        return np.zeros((0, 3), dtype=np.float32)
    return np.concatenate(rings).astype(np.float32)


def person_disks(people, radius, spacing=0.1, height=0.5, exclude=None, exclusion_radius=0.0):
    """
    Turn each person (x, y) into a filled disk of points `spacing` apart, `radius` across.

    For the costmaps: the planner treats the whole disk as an obstacle, so the
    path keeps that much room around a person. People closer than
    `exclusion_radius` to `exclude` (x, y) are left out. Returns Nx3 float32.
    """
    steps = np.arange(-radius, radius + 1e-9, spacing)
    gx, gy = np.meshgrid(steps, steps)
    keep = gx * gx + gy * gy <= radius * radius + 1e-9
    offsets = np.stack([gx[keep], gy[keep]], axis=1)
    disks = []
    for x, y in people:
        if exclude is not None and math.hypot(x - exclude[0], y - exclude[1]) < exclusion_radius:
            continue
        disks.append(np.column_stack([offsets + (x, y), np.full(len(offsets), height)]))
    if not disks:
        return np.zeros((0, 3), dtype=np.float32)
    return np.concatenate(disks).astype(np.float32)


def clearing_ring(reach, spacing=0.05, height=0.5):
    """
    Get points on a circle of radius `reach` around the origin, at most `spacing` apart.

    Raytraced from the robot by a clearing-only costmap source, they wipe
    everything within `reach` in the people layer each update, before the
    current people are marked again: nobody who has left stays behind.
    """
    count = max(8, int(math.ceil(2.0 * math.pi * reach / spacing)))
    angles = 2.0 * math.pi * np.arange(count) / count
    return np.stack([reach * np.cos(angles), reach * np.sin(angles),
                     np.full(count, height)], axis=1).astype(np.float32)


def in_view(point, rotation, translation, K, width, height, max_range=6.0):
    """
    Tell whether a point (camera-frame transform given) falls inside the camera's view.

    Only the horizontal field of view and the range are checked: a low camera
    sees a person's legs even when their head is above the picture.
    """
    p = np.asarray(rotation) @ np.asarray(point, dtype=float) + np.asarray(translation)
    if not 0.3 < p[2] < max_range:
        return False
    u = K[0] * p[0] / p[2] + K[2]
    return 0.0 <= u <= width


def quat_to_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
