"""The destination the web server gives the detector: set while driving, cleared after."""
import os
import time
from types import SimpleNamespace

import pytest

# Never talk to a running robot: a private ROS domain, on this machine only.
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402, I100
from rover_web.ros_side import CANCELED, LATCHED, RosSide, SUCCEEDED  # noqa: E402

FAILED = 6


@pytest.fixture
def ros():
    rclpy.init()
    node = RosSide()
    node.calls = []
    node.set_destination = lambda x, y: node.calls.append(('set', x, y))
    node.clear_destination = lambda: node.calls.append(('clear',))
    node._send = lambda *args, **kwargs: True  # no Nav2 here: pretend it took the goal
    node.event = lambda level, text: None
    yield node
    node.destroy_node()
    rclpy.shutdown()


def finish(node, kind, status):
    node.nav = {'active': True, 'kind': kind, 'text': 'x', 'current': 0, 'total': 1,
                'remaining': None}
    result = SimpleNamespace(status=status, result=SimpleNamespace())
    node._on_result(node.goal_seq, SimpleNamespace(result=lambda: result))


def test_goto_sets_the_destination(ros):
    ros.goto(2.0, 3.0, 0.0, 'x', kind='table')
    assert ros.calls == [('set', 2.0, 3.0)]


def test_route_follows_the_current_waypoint(ros):
    ros.route([(1.0, 0.0, None), (2.0, 0.0, 0.0), (3.0, 1.0, None)], 0, 'r')
    assert ros.calls == [('set', 1.0, 0.0)]
    ros.nav = {'active': True, 'kind': 'route', 'current': 0}

    def feedback(i):
        ros._on_feedback(ros.goal_seq, 'route', SimpleNamespace(
            feedback=SimpleNamespace(current_waypoint=i)))

    feedback(0)
    feedback(1)
    feedback(1)
    feedback(2)
    feedback(0)  # looping
    assert ros.calls[1:] == [('set', 2.0, 0.0), ('set', 3.0, 1.0), ('set', 1.0, 0.0)]


@pytest.mark.parametrize('kind, status', [
    ('goto', SUCCEEDED), ('home', SUCCEEDED), ('route', SUCCEEDED),
    ('table', FAILED), ('table', CANCELED), ('goto', FAILED)])
def test_finished_goals_clear_the_destination(ros, kind, status):
    ros.on_table_arrived = lambda label: None
    finish(ros, kind, status)
    assert ros.calls == [('clear',)]


def test_reached_table_keeps_it_for_the_approach(ros):
    arrived = []
    ros.on_table_arrived = arrived.append
    finish(ros, 'table', SUCCEEDED)
    assert ros.calls == [] and arrived == ['x']


def test_reached_table_without_approach_clears(ros):
    ros.on_table_arrived = None
    finish(ros, 'table', SUCCEEDED)
    assert ros.calls == [('clear',)]


def test_cancel_clears(ros):
    ros.cancel()
    assert ros.calls == [('clear',)]


def test_destination_messages_reach_a_latched_subscriber():
    rclpy.init()
    try:
        web = RosSide()
        web.set_destination(1.5, -2.0)
        listener = rclpy.create_node('destination_listener')
        got = []
        listener.create_subscription(PoseStamped, 'perception/destination', got.append, LATCHED)

        def spin_until(n):
            deadline = time.monotonic() + 5.0
            while len(got) < n and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.05)
                rclpy.spin_once(web, timeout_sec=0.0)

        spin_until(1)  # published before the listener existed: latched
        assert got and got[-1].header.frame_id == 'map'
        assert (got[-1].pose.position.x, got[-1].pose.position.y) == (1.5, -2.0)
        web.clear_destination()
        spin_until(2)
        assert got[-1].header.frame_id == ''
        listener.destroy_node()
        web.destroy_node()
    finally:
        rclpy.shutdown()
