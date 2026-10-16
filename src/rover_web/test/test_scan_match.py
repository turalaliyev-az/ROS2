import math

import numpy as np
from rover_web.scan_match import carry, localize, localize_near, MapField, thin

RES = 0.05


def building():
    """Build a 12 x 8 m floor: outer walls, an inner wall with a door, pillars, a table."""
    w, h = 240, 160
    g = np.zeros((h, w), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:, 140] = 100
    g[60:80, 140] = 0                      # door
    for cx, cy in ((40, 40), (100, 120), (190, 50)):
        g[cy - 4:cy + 4, cx - 4:cx + 4] = 100
    g[100:110, 30:60] = 100                # table
    return MapField(RES, w, h, 0.0, 0.0, g.ravel())


def scan_at(field, x, y, th, noise=0.01, seed=0):
    """Simulate a 360-degree lidar from pose (x, y, th): points in the robot frame."""
    rng = np.random.default_rng(seed)
    pts = []
    for a in np.radians(np.arange(0.0, 360.0, 0.8)):
        for r in np.arange(0.1, 12.0, 0.02):
            wx, wy = x + r * math.cos(th + a), y + r * math.sin(th + a)
            i, j = int(wx / RES), int(wy / RES)
            if not (0 <= i < field.grid.shape[1] and 0 <= j < field.grid.shape[0]):
                break
            if field.grid[j, i] >= 65:
                rr = r + rng.normal(0, noise)
                pts.append((rr * math.cos(a), rr * math.sin(a)))
                break
    return np.array(pts)


FIELD = building()


def test_global_search_finds_the_pose():
    true = (3.1, 2.3, math.radians(37))
    res = localize(FIELD, thin(scan_at(FIELD, *true)))
    assert res['trusted']
    assert math.hypot(res['x'] - true[0], res['y'] - true[1]) < 0.06
    assert abs(math.degrees(math.remainder(res['yaw'] - true[2], 2 * math.pi))) < 2.0


def test_local_search_around_a_rough_pose():
    true = (9.0, 5.5, math.radians(-120))
    rough = (9.5, 5.2)                      # saved pose 0.6 m and 25 deg off
    res = localize(FIELD, thin(scan_at(FIELD, *true)), center=rough, radius=1.5,
                   yaw=math.radians(-95), yaw_window=40)
    assert res['trusted']
    assert math.hypot(res['x'] - true[0], res['y'] - true[1]) < 0.06


def empty_room():
    """Build an empty 6 x 4 m room: it looks the same turned by 180 degrees."""
    w, h = 120, 80
    g = np.zeros((h, w), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    return MapField(RES, w, h, 0.0, 0.0, g.ravel())


def test_symmetric_place_is_not_trusted():
    room = empty_room()
    res = localize(room, thin(scan_at(room, 2.0, 1.5, 0.3)))
    assert not res['trusted']


def test_hint_settles_a_symmetric_place():
    room = empty_room()
    pts = thin(scan_at(room, 2.0, 1.5, 0.3))
    res = localize_near(room, pts, (2.2, 1.4, 0.5))
    assert res['trusted'] and res['by_hint']
    assert math.hypot(res['x'] - 2.0, res['y'] - 1.5) < 0.06
    # a hint at the mirror pose picks the mirror: the scan can't tell them apart
    mirror = localize_near(room, pts, (4.0, 2.5, 0.3 + math.pi))
    assert mirror['trusted'] and math.hypot(mirror['x'] - 4.0, mirror['y'] - 2.5) < 0.06


def test_stale_hint_does_not_override_a_clear_answer():
    # local search near (3, 2) alone finds a fair but wrong fit there
    true = (9.0, 5.5, math.radians(-120))
    res = localize_near(FIELD, thin(scan_at(FIELD, *true)), (3.0, 2.0, 0.0))
    assert res['trusted'] and not res['by_hint']
    assert math.hypot(res['x'] - true[0], res['y'] - true[1]) < 0.06


def test_scan_from_another_place_is_not_trusted():
    # robot in a small square room that isn't on the map
    w, h = 60, 60
    g = np.zeros((h, w), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[20:24, 35:45] = 100
    elsewhere = MapField(RES, w, h, 0.0, 0.0, g.ravel())
    res = localize(FIELD, thin(scan_at(elsewhere, 1.0, 1.5, 0.3)))
    assert not res['trusted']


def test_map_without_walls_gives_nothing_trusted():
    blank = MapField(RES, 100, 100, 0.0, 0.0, np.zeros(100 * 100, dtype=np.int16))
    res = localize(blank, thin(scan_at(FIELD, 3.1, 2.3, 0.4)))
    assert res is None or not res['trusted']


def test_carry_moves_the_pose_by_odometry():
    # odometry: 1 m forward along odom's y axis, then a quarter turn left
    odom_then, odom_now = (2.0, 1.0, math.pi / 2), (2.0, 2.0, math.pi)
    x, y, th = carry((5.0, 5.0, 0.0), odom_then, odom_now)   # map heading is +x
    assert math.isclose(x, 6.0, abs_tol=1e-9) and math.isclose(y, 5.0, abs_tol=1e-9)
    assert math.isclose(th, math.pi / 2, abs_tol=1e-9)
    assert np.allclose(carry((1.0, 2.0, 0.3), odom_then, odom_then), (1.0, 2.0, 0.3))
