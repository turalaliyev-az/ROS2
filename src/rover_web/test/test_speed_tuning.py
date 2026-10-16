"""The web server keeps Nav2's turn regulation in step with the speed slider."""
import os
from types import SimpleNamespace

import pytest

# Never talk to a running robot: a private ROS domain, on this machine only.
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

rclpy = pytest.importorskip('rclpy')
from rclpy.task import Future  # noqa: E402, I100
from rover_bringup import profile  # noqa: E402
from rover_web.ros_side import RosSide  # noqa: E402

PARAM = 'FollowPath.regulated_linear_scaling_min_radius'


class FakeParams:
    """controller_server's parameter services, answered by the test."""

    def __init__(self):
        self.ready = False
        self.calls = []  # (name, value, future)

    def services_are_ready(self):
        return self.ready

    def set_parameters(self, parameters):
        future = Future()
        for p in parameters:
            self.calls.append((p.name, p.value, future))
        return future


def answer(future, ok=True):
    future.set_result(SimpleNamespace(results=[SimpleNamespace(successful=ok)]))


@pytest.fixture
def ros():
    rclpy.init()
    node = RosSide()
    node.controller_params = FakeParams()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_waits_for_the_controller(ros):
    ros._publish_speed_limit()
    assert ros.controller_params.calls == [] and ros.tuned_speed is None


def test_tunes_for_the_slider_speed_once(ros):
    params = ros.controller_params
    params.ready = True
    ros.nav_speed = 1.39
    ros._publish_speed_limit()
    ros._publish_speed_limit()  # still waiting for the answer: no second call
    assert [(n, v) for n, v, _ in params.calls] == [(PARAM, profile.min_turn_radius(1.39))]
    answer(params.calls[0][2])
    assert ros.tuned_speed == 1.39
    ros._publish_speed_limit()
    assert len(params.calls) == 1


def test_slow_speed_gets_the_original_tuning_back(ros):
    params = ros.controller_params
    params.ready = True
    ros.nav_speed = 1.39
    ros._publish_speed_limit()
    answer(params.calls[-1][2])
    ros.nav_speed = 0.2
    ros._publish_speed_limit()
    assert params.calls[-1][:2] == (PARAM, 0.5)


def test_a_refused_change_is_retried(ros):
    params = ros.controller_params
    params.ready = True
    ros.nav_speed = 0.8
    ros._publish_speed_limit()
    answer(params.calls[-1][2], ok=False)  # e.g. the plugin is not configured yet
    assert ros.tuned_speed is None
    ros._publish_speed_limit()
    assert len(params.calls) == 2


def test_a_restarted_controller_is_tuned_again(ros):
    params = ros.controller_params
    params.ready = True
    ros.nav_speed = 1.0
    ros._publish_speed_limit()
    answer(params.calls[-1][2])
    params.ready = False   # mode change: controller_server goes away...
    ros._publish_speed_limit()
    params.ready = True    # ...and comes back with the file's values
    ros._publish_speed_limit()
    assert len(params.calls) == 2 and params.calls[-1][1] == profile.min_turn_radius(1.0)
