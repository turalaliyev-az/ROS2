import asyncio
import math
import threading
from types import SimpleNamespace

import pytest
from rover_web.approach import TableApproach

FRONT = 0.3  # m from the rotation centre to the robot's front
STEP = 0.05  # s of simulated motion per control step


class FakeRos:
    """The parts of RosSide the approach uses, with a robot that moves as commanded."""

    def __init__(self, table=(0.9, 0.15), moves=True):
        self.lock = threading.Lock()
        self.nav = {'active': False, 'kind': '', 'text': ''}
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.cmd = (0.0, 0.0)
        self.table = table   # near side of the table, odom; None = not in view
        self.moves = moves
        self.odom_ok = True
        self.events = []
        self.cleared = 0
        self.commands = 0
        self.objects_error = None

    def drive(self, v, w):
        self.cmd = (v, w)
        self.commands += 1

    def odom_pose(self):
        if not self.odom_ok:
            return None
        if self.moves:
            v, w = self.cmd
            self.pose['yaw'] += w * STEP
            self.pose['x'] += v * math.cos(self.pose['yaw']) * STEP
            self.pose['y'] += v * math.sin(self.pose['yaw']) * STEP
        return dict(self.pose)

    def table_in_base(self):
        dx, dy = self.table[0] - self.pose['x'], self.table[1] - self.pose['y']
        c, s = math.cos(self.pose['yaw']), math.sin(self.pose['yaw'])
        return c * dx + s * dy, -s * dx + c * dy

    def latest_objects(self, max_age):
        if self.objects_error:
            raise self.objects_error
        if self.table is None:
            return {'objects': [{'cls': 'person', 'base': [1.0, 0.0]}]}
        bx, by = self.table_in_base()
        return {'objects': [{'cls': 'chair', 'base': [0.5, 0.0]},
                            {'cls': 'table', 'base': [round(bx, 3), round(by, 3)]}]}

    def event(self, level, text):
        self.events.append((level, text))

    def clear_destination(self):
        self.cleared += 1


def make(ros, approach=True, gap=0.2):
    store = SimpleNamespace(settings={'table_approach': approach, 'table_gap': gap})
    a = TableApproach(ros, store, lambda: FRONT)
    a.settle = a.pause = 0.0
    a.step = 0.001
    return a


def run(a, seq=None):
    asyncio.run(a.run('5', a.seq if seq is None else seq))


def assert_finished(ros):
    assert ros.cmd == (0.0, 0.0)
    assert ros.cleared == 1
    assert not ros.nav['active']


def test_turns_to_the_table_and_stops_at_the_gap():
    ros = FakeRos(table=(0.9, 0.15))
    run(make(ros, gap=0.2))
    bx, by = ros.table_in_base()
    assert abs(math.atan2(by, bx)) < 0.04          # facing it
    assert bx - FRONT == pytest.approx(0.2, abs=0.015)  # table_gap from the front
    assert len(ros.events) == 1
    level, text = ros.events[0]
    assert level == 'info' and text.startswith('Çatdı: Masa 5 (masaya qədər')
    assert_finished(ros)


def test_backs_off_when_too_close():
    ros = FakeRos(table=(0.4, 0.0))
    run(make(ros, gap=0.2))
    assert ros.table_in_base()[0] - FRONT == pytest.approx(0.2, abs=0.015)
    assert ros.pose['x'] < 0.0
    assert_finished(ros)


def test_disabled_just_reports_arrival():
    ros = FakeRos()
    run(make(ros, approach=False))
    assert ros.events == [('info', 'Çatdı: Masa 5')]
    assert ros.commands == 0
    assert ros.cleared == 1


def test_table_not_in_view_reports_and_stays():
    ros = FakeRos(table=None)
    run(make(ros))
    assert ros.pose == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    assert len(ros.events) == 1 and 'görünmədi' in ros.events[0][1]
    assert_finished(ros)


def test_blocked_move_times_out_with_a_warning():
    ros = FakeRos(table=(0.9, 0.0), moves=False)  # e.g. the collision monitor holds it
    a = make(ros)
    a.drive_timeout = lambda distance: 0.2
    run(a)
    assert len(ros.events) == 1
    level, text = ros.events[0]
    assert level == 'warn' and 'tamamlanmadı' in text
    assert_finished(ros)


def test_blocked_turn_times_out_with_a_warning():
    ros = FakeRos(table=(0.9, 0.4), moves=False)
    a = make(ros)
    a.turn_timeout = 0.2
    run(a)
    assert ros.events[0][0] == 'warn'
    assert_finished(ros)


def test_lost_odometry_stops_with_an_error():
    ros = FakeRos(table=(0.9, 0.0))
    ros.odom_ok = False
    run(make(ros))
    assert len(ros.events) == 1
    level, text = ros.events[0]
    assert level == 'error' and 'odometriya' in text
    assert_finished(ros)


def test_unexpected_error_is_reported_and_the_robot_stopped():
    ros = FakeRos()
    ros.objects_error = RuntimeError('boom')
    run(make(ros))
    assert len(ros.events) == 1
    level, text = ros.events[0]
    assert level == 'error' and 'boom' in text
    assert_finished(ros)


def test_cancel_mid_move_is_silent_and_hands_over():
    ros = FakeRos(table=(0.9, 0.0), moves=False)
    a = make(ros)
    a.drive_timeout = lambda distance: 5.0

    async def scenario():
        task = asyncio.ensure_future(a.run('5', a.seq))
        while not ros.commands:
            await asyncio.sleep(0.001)
        with ros.lock:
            assert ros.nav == {'active': True, 'kind': 'approach', 'text': 'Masa 5: dəqiq yanaşma',
                               'current': 0, 'total': 1, 'remaining': None}
        a.cancel()  # e.g. STOP pressed
        await asyncio.wait_for(task, 1.0)

    asyncio.run(scenario())
    assert ros.events == []
    assert_finished(ros)


def test_cancelled_before_it_started_does_nothing():
    ros = FakeRos()
    a = make(ros)
    seq = a.seq
    a.cancel()  # a command came in between Nav2's result and the approach starting
    run(a, seq=seq)
    assert ros.events == [] and ros.commands == 0 and ros.cleared == 0


def test_cancel_when_idle_leaves_navigation_alone():
    ros = FakeRos()
    ros.nav = {'active': True, 'kind': 'goto', 'text': 'x'}
    make(ros).cancel()
    assert ros.nav['active'] and ros.cleared == 0
