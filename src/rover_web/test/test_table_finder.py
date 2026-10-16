import math

from rover_web.table_finder import (add_sighting, approach_pose, MIN_SIGHTING_INTERVAL,
                                    MIN_SIGHTINGS, offered, prune, STALE_AGE, table_center)


def sight(cands, center, robot=(1.0, 1.0), radius=0.4, now=0.0):
    return add_sighting(cands, center, radius, robot, next_id=len(cands) + 1, now=now)


def test_sightings_from_one_side_make_one_candidate():
    cands = []
    # A 0.8 m table centred at (3, 1); the robot sees its near side from the west.
    for i in range(6):
        robot = (1.0, 1.0 + 0.05 * (i - 3))
        near = (2.6, 1.0 + 0.02 * (i % 2))
        center, r = table_center(near, robot, width=0.8)
        add_sighting(cands, center, r, robot, next_id=len(cands) + 1, now=float(i))
    assert len(cands) == 1
    c = cands[0]
    assert abs(c['x'] - 3.0) < 0.05 and abs(c['y'] - 1.0) < 0.05
    assert offered(cands, {}) == [c]
    x, y, yaw = approach_pose(c, robot_front=0.3, gap=0.25)
    # stops west of the table, facing east
    assert x < 3.0 - 0.4 - 0.3 and abs(y - 1.0) < 0.1
    assert abs(yaw) < 0.1


def test_a_burst_of_detections_counts_once_per_interval():
    cands = []
    # 8 Hz for 2 s: 16 detections, but only about 2 s worth of sightings.
    for i in range(16):
        sight(cands, (3.0, 1.0), now=i / 8.0)
    assert len(cands) == 1
    assert cands[0]['n'] == 2
    assert offered(cands, {}) == []
    # Seen long enough: offered.
    for k in range(MIN_SIGHTINGS):
        sight(cands, (3.0, 1.0), now=2.0 + k * MIN_SIGHTING_INTERVAL)
    assert offered(cands, {}) == cands


def test_known_and_dismissed_are_not_offered():
    cands = []
    for i in range(5):
        sight(cands, (3.0, 1.0), now=float(i))
    assert offered(cands, {'5': {'x': 2.2, 'y': 1.0, 'yaw': 0.0}}) == []
    assert offered(cands, {'5': {'x': 9, 'y': 9, 'yaw': 0, 'center': [3.1, 1.0]}}) == []
    cands[0]['dismissed'] = True
    assert offered(cands, {}) == []


def test_far_sightings_are_separate_tables():
    cands = []
    sight(cands, (3.0, 1.0), robot=(1.0, 1.0))
    sight(cands, (3.0, 2.5), robot=(1.0, 2.5))
    assert len(cands) == 2
    assert math.isclose(cands[0]['vx'], -1.0)


def test_prune_forgets_only_stale_unconfirmed_candidates():
    cands = []
    for i in range(MIN_SIGHTINGS):
        sight(cands, (3.0, 1.0), now=float(i))          # confirmed
    sight(cands, (8.0, 1.0), now=0.0)                    # seen once, long ago
    sight(cands, (8.0, 5.0), now=0.0)['dismissed'] = True
    sight(cands, (1.0, 5.0), now=STALE_AGE)              # seen once, recently
    legacy = {'id': 99, 'x': 0.0, 'y': -5.0, 'r': 0.4, 'vx': 1.0, 'vy': 0.0, 'n': 1,
              'dismissed': False}                         # saved before 'last' existed
    cands.append(legacy)
    prune(cands, now=STALE_AGE + 10.0)
    assert [(c['x'], c['y']) for c in cands] == [(3.0, 1.0), (8.0, 5.0), (1.0, 5.0)]
