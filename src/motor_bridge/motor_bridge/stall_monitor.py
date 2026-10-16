"""
Drive fault detection: a wheel told to move that doesn't (no ROS).

A stuck or blocked wheel, a dead motor or a broken wire all look the same
from here: the wheel's commanded speed is well above zero, its measured speed
stays far below it. After `hold_s` of that the drive is faulted: the bridge
stops the motors and reports it, and the fault clears only once the commanded
speed has been zero for `clear_s` (someone stopped the robot on purpose).
"""


class StallMonitor:
    """Watches both wheels; speeds in m/s, wheel-frame forward-positive."""

    def __init__(self, min_speed=0.05, ratio=0.2, hold_s=1.5, clear_s=0.5):
        self.min_speed = min_speed
        self.ratio = ratio
        self.hold_s = hold_s
        self.clear_s = clear_s
        self.stalled_s = {'left': 0.0, 'right': 0.0}
        self.idle_s = 0.0
        self.fault = None   # dict describing the fault while latched

    def update(self, target_l, target_r, measured_l, measured_r, dt):
        """Feed one control step; returns the latched fault (or None)."""
        if self.fault is not None:
            if abs(target_l) < 1e-6 and abs(target_r) < 1e-6:
                self.idle_s += dt
                if self.idle_s >= self.clear_s:
                    self.fault = None
                    self.idle_s = 0.0
                    self.stalled_s = {'left': 0.0, 'right': 0.0}
            else:
                self.idle_s = 0.0
            return self.fault
        for wheel, target, measured in (('left', target_l, measured_l),
                                        ('right', target_r, measured_r)):
            moving_wanted = abs(target) >= self.min_speed
            along = measured * (1.0 if target >= 0 else -1.0)  # speed in the commanded direction
            if moving_wanted and along < self.ratio * abs(target):
                self.stalled_s[wheel] += dt
            else:
                self.stalled_s[wheel] = 0.0
            if self.stalled_s[wheel] >= self.hold_s:
                self.fault = {'wheel': wheel, 'target': round(target, 3),
                              'measured': round(measured, 3),
                              'seconds': round(self.stalled_s[wheel], 2)}
                return self.fault
        return None
