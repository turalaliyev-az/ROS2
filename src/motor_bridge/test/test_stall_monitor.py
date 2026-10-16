from motor_bridge.stall_monitor import StallMonitor

DT = 0.05


def run(monitor, seconds, target_l, target_r, measured_l, measured_r):
    fault = None
    for _ in range(round(seconds / DT)):
        fault = monitor.update(target_l, target_r, measured_l, measured_r, DT)
    return fault


def test_following_wheels_never_fault():
    m = StallMonitor()
    assert run(m, 10, 0.2, 0.2, 0.19, 0.21) is None
    assert run(m, 10, -0.3, 0.3, -0.25, 0.28) is None


def test_slow_commands_and_rest_are_ignored():
    m = StallMonitor()
    assert run(m, 10, 0.03, 0.03, 0.0, 0.0) is None   # below the minimum: may legitimately sit
    assert run(m, 10, 0.0, 0.0, 0.0, 0.0) is None


def test_a_stuck_wheel_faults_after_the_hold_time():
    m = StallMonitor(hold_s=1.5)
    assert run(m, 1.4, 0.2, 0.2, 0.2, 0.0) is None
    fault = run(m, 0.2, 0.2, 0.2, 0.2, 0.0)
    assert fault['wheel'] == 'right' and fault['target'] == 0.2 and fault['measured'] == 0.0


def test_wrong_direction_counts_as_stuck():
    m = StallMonitor()
    assert run(m, 2.0, 0.2, 0.2, -0.1, 0.2)['wheel'] == 'left'


def test_brief_stalls_reset():
    m = StallMonitor(hold_s=1.5)
    for _ in range(5):
        assert run(m, 1.0, 0.2, 0.2, 0.0, 0.2) is None   # stuck for 1 s ...
        assert run(m, 0.2, 0.2, 0.2, 0.2, 0.2) is None   # ... then rolls again


def test_fault_latches_until_commanded_to_zero():
    m = StallMonitor(clear_s=0.5)
    assert run(m, 2.0, 0.2, 0.2, 0.0, 0.0) is not None
    # still commanded: stays faulted even though nothing is measured
    assert run(m, 5.0, 0.2, 0.2, 0.0, 0.0) is not None
    assert run(m, 0.3, 0.0, 0.0, 0.0, 0.0) is not None   # not long enough at zero
    assert run(m, 0.3, 0.0, 0.0, 0.0, 0.0) is None
    assert run(m, 1.0, 0.2, 0.2, 0.2, 0.2) is None        # driving normally again
