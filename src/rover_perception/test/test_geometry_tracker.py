import math

import numpy as np
import pytest
from rover_perception.geometry import (clearing_ring, depth_points, in_view, locate,
                                       person_disks, person_rings, project, quat_to_matrix,
                                       scale_intrinsics, transform)
from rover_perception.tracker import Tracker

# D435-like: depth 848x480 wide, colour 640x480 narrower, 1.5 cm apart.
K_DEPTH = [420.0, 0, 424.0, 0, 420.0, 240.0, 0, 0, 1]
K_COLOR = [610.0, 0, 320.0, 0, 610.0, 240.0, 0, 0, 1]
DEPTH_TO_COLOR = (np.eye(3), [0.015, 0.0, 0.0])


def scene(back_depth=None):
    """
    Build a wall 3 m away with an object 1.2 m away, 0.3 m right of the optical axis.

    back_depth: the object's right half is that far instead (a deeper object
    seen at an angle, like a table whose top recedes from the camera).
    """
    depth = np.full((480, 848), 3.0, dtype=np.float32)
    # object occupies x in [0.15, 0.45] m, y in [-0.3, 0.3] m at z = 1.2
    u1, u2 = int(424 + 0.15 / 1.2 * 420), int(424 + 0.45 / 1.2 * 420)
    v1, v2 = int(240 - 0.3 / 1.2 * 420), int(240 + 0.3 / 1.2 * 420)
    depth[v1:v2, u1:u2] = 1.2
    if back_depth is not None:
        depth[v1:v2, (u1 + u2) // 2:u2] = back_depth
    depth[:20, :] = 0.0  # no readings in a strip, as on shiny surfaces
    return depth


def colour_box(x1, x2, y1, y2, z, offset=0.015):
    """Give where a box of the given camera-frame extent shows up in the colour image."""
    def px(x, y):
        return K_COLOR[0] * (x + offset) / z + K_COLOR[2], K_COLOR[4] * y / z + K_COLOR[5]
    a, b = px(x1, y1), px(x2, y2)
    return [a[0], a[1], b[0], b[1]]


def located(depth):
    pts = transform(depth_points(depth, K_DEPTH, stride=2), *DEPTH_TO_COLOR)
    pixels, front = project(pts, K_COLOR)
    # a detector box slightly larger than the object: wall pixels around it
    box = colour_box(0.10, 0.50, -0.35, 0.35, 1.2)
    return locate(pts[front], pixels[front], box)


def test_locate_finds_object_not_wall():
    centre, surface, near = located(scene())
    assert abs(near - 1.2) < 0.02
    assert abs(centre[2] - 1.2) < 0.02
    assert abs(centre[0] - (0.30 + 0.015)) < 0.05
    assert np.allclose(surface, centre, atol=0.02)


def test_locate_surface_is_the_near_side_of_a_deep_object():
    centre, surface, near = located(scene(back_depth=1.5))
    assert abs(near - 1.2) < 0.02
    # The median sits between front and back; the surface point is at the front,
    # on the same line of sight.
    assert centre[2] > 1.3
    assert abs(surface[2] - near) < 1e-6
    assert np.allclose(surface / np.linalg.norm(surface), centre / np.linalg.norm(centre))


def test_locate_none_without_depth():
    pts = np.zeros((0, 3))
    assert locate(pts, np.zeros((0, 2)), [0, 0, 100, 100]) is None


def test_scale_intrinsics_follows_decimation():
    k = scale_intrinsics(K_DEPTH, (848, 480), (283, 160))
    assert k[0] == pytest.approx(420.0 * 283 / 848)
    assert k[4] == pytest.approx(420.0 / 3)
    # The image centre stays the image centre.
    assert (k[2] + 0.5) / 283 == pytest.approx((K_DEPTH[2] + 0.5) / 848)
    assert scale_intrinsics(K_DEPTH, (848, 480), (848, 480)) == K_DEPTH
    assert scale_intrinsics(K_DEPTH, (0, 0), (283, 160)) == K_DEPTH


def test_decimated_depth_with_scaled_intrinsics_locates_the_same_object():
    full = scene()
    small = full[1::3, 1::3]
    k_small = scale_intrinsics(K_DEPTH, (848, 480), (small.shape[1], small.shape[0]))
    pts = transform(depth_points(small, k_small, stride=1), *DEPTH_TO_COLOR)
    pixels, front = project(pts, K_COLOR)
    box = colour_box(0.10, 0.50, -0.35, 0.35, 1.2)
    centre, _, near = locate(pts[front], pixels[front], box)
    assert abs(near - 1.2) < 0.02
    assert abs(centre[0] - (0.30 + 0.015)) < 0.05


def test_person_rings_and_destination_exclusion():
    people = [(1.0, 0.0), (5.0, 5.0)]
    pts = person_rings(people, radius=0.3, count=12, height=0.5)
    assert pts.shape == (24, 3) and pts.dtype == np.float32
    first = pts[:12]
    assert np.allclose(np.hypot(first[:, 0] - 1.0, first[:, 1]), 0.3, atol=1e-5)
    assert np.allclose(pts[:, 2], 0.5)
    # The customer at the destination is left out, the other person stays.
    kept = person_rings(people, 0.3, 12, 0.5, exclude=(5.2, 5.0), exclusion_radius=1.2)
    assert kept.shape == (12, 3)
    assert np.allclose(kept[:, :2].mean(axis=0), (1.0, 0.0), atol=1e-5)
    assert person_rings([], 0.3).shape == (0, 3)


def test_quat_to_matrix_yaw():
    yaw = 0.7
    r = quat_to_matrix(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert np.allclose(r @ [1.0, 0.0, 0.0], [math.cos(yaw), math.sin(yaw), 0.0])


def test_tracker_keeps_ids_and_drops_old():
    tr = Tracker(match_radius=0.6, max_age=1.0)
    a = {'cls': 'person', 'pos': (1.0, 0.0)}
    b = {'cls': 'person', 'pos': (3.0, 0.0)}
    c = {'cls': 'chair', 'pos': (1.1, 0.0)}
    tr.update([a, b, c], now=0.0)
    ids = (a['id'], b['id'], c['id'])
    assert len(set(ids)) == 3
    a2 = {'cls': 'person', 'pos': (1.2, 0.0)}
    b2 = {'cls': 'person', 'pos': (3.1, 0.1)}
    tr.update([b2, a2], now=0.2)
    assert (a2['id'], b2['id']) == ids[:2]
    assert tr.tracks[a2['id']]['speed'] > 0.2  # smoothed: half of the 0.5 m/s step
    tracks = tr.update([], now=2.0)
    assert tracks == {}


def test_tracker_does_not_match_across_classes():
    tr = Tracker(match_radius=0.6, max_age=1.0)
    chair = {'cls': 'chair', 'pos': (1.0, 0.0)}
    tr.update([chair], now=0.0)
    person = {'cls': 'person', 'pos': (1.0, 0.0)}
    tr.update([person], now=0.1)
    assert person['id'] != chair['id']


def test_person_disks_fill_the_radius_and_skip_the_destination():
    pts = person_disks([(2.0, 1.0), (6.0, 0.0)], radius=0.5, spacing=0.1, height=0.5,
                       exclude=(6.3, 0.0), exclusion_radius=1.2)
    assert pts.dtype == np.float32 and np.allclose(pts[:, 2], 0.5)
    d = np.hypot(pts[:, 0] - 2.0, pts[:, 1] - 1.0)
    assert d.max() <= 0.5 + 1e-5          # only the first person, inside the radius
    assert d.min() < 1e-5                  # centre included: a filled disk, not a ring
    assert len(pts) > 70                   # ~pi * 5^2 points at 0.1 m
    assert person_disks([], 0.5).shape == (0, 3)


def test_clearing_ring_is_dense_enough_at_its_reach():
    ring = clearing_ring(6.0, spacing=0.05)
    assert np.allclose(np.hypot(ring[:, 0], ring[:, 1]), 6.0, atol=1e-4)
    gaps = np.hypot(*(np.diff(ring[:, :2], axis=0).T))
    assert gaps.max() <= 0.05 + 1e-6


def test_in_view_checks_horizontal_field_and_range():
    # optical frame: z forward, x right; camera looking along the robot's x axis
    to_cam = (np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float), np.zeros(3))
    assert in_view((2.0, 0.0, 0.3), *to_cam, K_COLOR, 640, 480)
    assert not in_view((2.0, 3.0, 0.3), *to_cam, K_COLOR, 640, 480)   # far to the left
    assert not in_view((-2.0, 0.0, 0.3), *to_cam, K_COLOR, 640, 480)  # behind
    assert in_view((2.0, 0.0, 1.8), *to_cam, K_COLOR, 640, 480)       # head above the picture
