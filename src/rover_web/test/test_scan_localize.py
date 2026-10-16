"""Controller._scan_localize against a simulated lidar and map (no ROS graph)."""
import asyncio
import math
import threading
from types import SimpleNamespace

from rover_web.server import Controller
from test_scan_match import FIELD, RES, scan_at


class FakeRos:
    """Just what _scan_localize reads from RosSide."""

    def __init__(self, true_pose, odom=((0.0, 0.0, 0.0),), scans=True):
        self.lock = threading.Lock()
        self.velocity = (0.0, 0.0)
        h, w = FIELD.grid.shape
        self.map_msg = SimpleNamespace(
            info=SimpleNamespace(resolution=RES, width=w, height=h,
                                 origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))),
            data=FIELD.grid.ravel().tolist())
        self.map_version = 1
        self.pts = scan_at(FIELD, *true_pose) if scans else None
        self.odom = list(odom)

    def lidar_points(self, since):
        return None if self.pts is None else (self.pts, 10)

    def odom_pose(self):
        x, y, yaw = self.odom.pop(0) if len(self.odom) > 1 else self.odom[0]
        return {'x': x, 'y': y, 'yaw': yaw}


def run(ros, hint=None):
    ctl = SimpleNamespace(ros=ros, robot_geometry={'inscribed_radius': 0.25},
                          _loc_field=(None, None))
    return asyncio.run(Controller._scan_localize(ctl, hint))


TRUE = (9.0, 5.5, math.radians(-120))


def off(found, pose=TRUE):
    return math.hypot(found['x'] - pose[0], found['y'] - pose[1])


def test_saved_pose_close_to_the_truth_is_confirmed():
    found = run(FakeRos(TRUE), hint=(9.3, 5.3, math.radians(-105)))
    assert found['trusted'] and off(found) < 0.06
    assert found['hint_hits'] < found['hits']


def test_wrong_saved_pose_is_replaced_by_a_map_search():
    found = run(FakeRos(TRUE), hint=(3.0, 2.0, 0.0))     # robot was carried elsewhere
    assert found['trusted'] and off(found) < 0.06
    assert found['hint_hits'] < 0.3


def test_motion_during_the_search_is_carried_forward():
    # odometry: 0.5 m forward between taking the scan and the answer
    ros = FakeRos(TRUE, odom=((1.0, 1.0, 0.0), (1.5, 1.0, 0.0)))
    found = run(ros)
    ahead = (TRUE[0] + 0.5 * math.cos(TRUE[2]), TRUE[1] + 0.5 * math.sin(TRUE[2]))
    assert found['trusted'] and off(found, ahead) < 0.06


def test_no_scan_gives_no_answer():
    assert run(FakeRos(TRUE, scans=False)) is None
