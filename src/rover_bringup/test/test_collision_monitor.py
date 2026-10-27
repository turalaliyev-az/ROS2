"""
Nav2's collision monitor with the parameters generated for this robot.

Starts the real collision_monitor node in a private ROS domain, feeds it a
clear lidar scan and a people cloud like rover_perception's detector sends,
and checks what it lets through from cmd_vel_smoothed to cmd_vel.
"""
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

import pytest

# Never talk to a running robot: a private ROS domain, on this machine only.
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist  # noqa: E402, I100
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rover_bringup import profile  # noqa: E402
from rover_perception.detector import make_cloud  # noqa: E402
from rover_perception.geometry import person_rings  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]
COMMAND = (0.2, 0.3)  # what "Nav2" asks for: m/s forward, rad/s left

pytestmark = pytest.mark.skipif(shutil.which('ros2') is None, reason='ROS 2 not sourced')


class Harness:
    """Stands in for the lidar, the detector, the costmap footprint and Nav2's controller."""

    def __init__(self, node, footprint):
        self.node = node
        self.lock = threading.Lock()
        self.people = []            # [(x, y)] in odom
        self.people_on = True       # False: the detector has gone silent
        self.command = COMMAND
        self.out = []               # (time, v, w) seen on cmd_vel
        self.states = []
        self.scan_pub = node.create_publisher(LaserScan, 'scan', 10)
        self.cloud_pub = node.create_publisher(PointCloud2, 'perception/people_cloud', 10)
        self.cmd_pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.fp_pub = node.create_publisher(PolygonStamped, 'local_costmap/published_footprint',
                                            10)
        node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        node.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self.states.append, 10)
        self.footprint = footprint
        self.tf = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        node.create_timer(0.05, self._tick)

    def _on_cmd(self, msg):
        with self.lock:
            self.out.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _tick(self):
        stamp = self.node.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = stamp
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = math.radians(1.0)
        scan.range_min, scan.range_max = 0.05, 12.0
        scan.ranges = [math.inf] * 360  # nothing around
        self.scan_pub.publish(scan)
        fp = PolygonStamped()
        fp.header.frame_id = 'base_footprint'
        fp.header.stamp = stamp
        fp.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in self.footprint]
        self.fp_pub.publish(fp)
        with self.lock:
            people, on, (v, w) = list(self.people), self.people_on, self.command
        if on:
            self.cloud_pub.publish(make_cloud(person_rings(people, 0.3), 'odom', stamp))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)

    def measure(self, people=(), command=COMMAND, people_on=True, settle=0.8, window=0.6):
        """Get the mean (v, w) let through once the situation has settled."""
        with self.lock:
            self.people, self.command, self.people_on = list(people), command, people_on
        time.sleep(settle)
        start = time.monotonic()
        time.sleep(window)
        with self.lock:
            seen = [(v, w) for t, v, w in self.out if t >= start]
        assert seen, 'collision monitor published nothing on cmd_vel'
        return (sum(v for v, _ in seen) / len(seen), sum(w for _, w in seen) / len(seen))


@pytest.fixture(scope='module')
def monitor(tmp_path_factory):
    params = profile.nav2_params(profile.DEFAULT, PACKAGE / 'config' / 'nav2_params.yaml',
                                 tmp_path_factory.mktemp('nav2') / 'nav2_params.yaml')
    proc = subprocess.Popen(
        ['ros2', 'run', 'nav2_collision_monitor', 'collision_monitor',
         '--ros-args', '--params-file', params],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('collision_monitor_test')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        harness = Harness(node, profile.derive(profile.DEFAULT)['footprint'])
        client = node.create_client(ChangeState, 'collision_monitor/change_state')
        assert client.wait_for_service(timeout_sec=20.0), 'collision_monitor did not start'
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            future = client.call_async(ChangeState.Request(transition=Transition(id=transition)))
            deadline = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            assert future.done() and future.result().success, \
                f'collision_monitor refused transition {transition} (bad parameters?)'
        yield harness
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


def test_nobody_around_full_command(monitor):
    v, w = monitor.measure()
    assert v == pytest.approx(COMMAND[0], abs=0.01)
    assert w == pytest.approx(COMMAND[1], abs=0.01)


def test_person_beside_caps_the_speed_at_walking_pace(monitor):
    # 1 m to the left: inside the 1.2 m circle. A cap, not a ratio: a slow
    # command passes untouched, a faster one is held to PEOPLE_NEAR_SPEED.
    v, w = monitor.measure(people=[(0.0, 1.0)])
    assert v == pytest.approx(COMMAND[0], abs=0.01)
    assert w == pytest.approx(COMMAND[1], abs=0.01)
    v, _ = monitor.measure(people=[(0.0, 1.0)], command=MEDIUM)
    assert v == pytest.approx(profile.PEOPLE_NEAR_SPEED, abs=0.01)
    assert any(s.polygon_name == 'PeopleSlowdown' for s in monitor.states)


def test_heading_into_someone_brakes(monitor):
    # Their nearest edge 5 cm beyond the robot's front, straight ahead.
    front = profile.derive(profile.DEFAULT)['footprint'][0][0]
    v, _ = monitor.measure(people=[(front + 0.35, 0.0)])
    assert abs(v) < 0.03
    assert any(s.polygon_name == 'PeopleApproach' for s in monitor.states)


def test_turning_in_place_beside_someone_is_not_held_back(monitor):
    # 0.4 m beyond the front: the robot's corners can't reach them in a turn,
    # so turning away (as the planner does to go round) goes ahead.
    front = profile.derive(profile.DEFAULT)['footprint'][0][0]
    v, w = monitor.measure(people=[(front + 0.7, 0.0)], command=(0.0, 0.3))
    assert abs(v) < 0.005
    assert w == pytest.approx(0.3, abs=0.02)


def test_passing_someone_beside_the_path_is_not_held_back(monitor):
    # Half a metre ahead and 0.8 m to the left: driving straight on misses
    # them, so only the walking-pace cap could apply (0.3 m/s is under it).
    front = profile.derive(profile.DEFAULT)['footprint'][0][0]
    v, _ = monitor.measure(people=[(front + 0.5, 0.8)], command=(0.3, 0.0))
    assert v == pytest.approx(0.3, abs=0.01)


def test_person_far_away_ignored(monitor):
    v, w = monitor.measure(people=[(3.0, 0.0)])
    assert v == pytest.approx(COMMAND[0], abs=0.01)


def test_silent_detector_stops_the_robot(monitor):
    # people source_timeout is 2 s: after that the monitor must not let commands through.
    v, w = monitor.measure(people_on=False, settle=2.6)
    assert abs(v) < 0.005 and abs(w) < 0.005
    # ...and recovers once the detector is back.
    v, _ = monitor.measure()
    assert v == pytest.approx(COMMAND[0], abs=0.01)


# ---------------- at 5 km/h ----------------

FAST = (1.3, 0.0)
MEDIUM = (0.6, 0.0)


def test_fast_with_nobody_around_full_command(monitor):
    v, _ = monitor.measure(command=FAST)
    assert v == pytest.approx(FAST[0], abs=0.02)


def test_someone_one_metre_ahead_brakes_a_fast_robot_only(monitor):
    # Their nearest edge 1 m beyond the front. At 0.2 m/s that is 5 s away:
    # nothing to do. Faster, the robot slows to keep 2 s between them, the
    # same speed (1 m / 2 s) whatever it was asked for.
    front = profile.derive(profile.DEFAULT)['footprint'][0][0]
    person = [(front + 1.0 + 0.3, 0.0)]
    slow, _ = monitor.measure(people=person, command=(0.2, 0.0))
    assert slow == pytest.approx(0.2, abs=0.01)
    keep = 1.0 / profile.PEOPLE_TIME_BEFORE_COLLISION
    medium, _ = monitor.measure(people=person, command=MEDIUM)
    assert medium == pytest.approx(keep, abs=0.06)
    fast, _ = monitor.measure(people=person, command=FAST)
    assert fast == pytest.approx(keep, abs=0.06)
