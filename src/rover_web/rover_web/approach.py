"""
Camera-guided last few centimetres at a table, after Nav2 has arrived.

Nav2 stops at the table's saved pose, which is only as good as the map and
the localization. The camera then sees the table itself: the robot turns to
face it and drives straight until its front is `table_gap` from it, slowly,
through the collision monitor (ros.drive), so the lidar and the people zones
still protect this move.

No ROS imports: `ros` is the RosSide, or a stand-in in the tests.
"""
import asyncio
import math
import time

OK = 'ok'
CANCELLED = 'cancelled'
TIMEOUT = 'timeout'
NO_ODOM = 'no_odom'


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TableApproach:
    """One approach at a time; any command that moves the robot cancels it first."""

    speed = 0.08            # m/s straight ahead
    turn_tolerance = 0.03   # rad
    turn_timeout = 8.0      # s
    settle = 0.8            # s: fresh detections, Nav2's last commands gone
    pause = 0.5             # s: after a move, before looking again
    step = 0.05             # s: control period

    def __init__(self, ros, store, robot_front):
        self.ros = ros
        self.store = store              # .settings: table_approach, table_gap
        self.robot_front = robot_front  # () -> m from the rotation centre to the front
        self.seq = 0
        self.active = False

    # ---------------- control ----------------

    def cancel(self):
        """Stop a running approach; whatever cancels it takes over the robot."""
        self.seq += 1
        if self.active:
            self._end()

    def _end(self):
        self.active = False
        with self.ros.lock:
            if self.ros.nav.get('kind') == 'approach':
                self.ros.nav['active'] = False
        self.ros.clear_destination()

    async def run(self, name, seq):
        """
        Approach table `name` and report the outcome as an event.

        `seq` is self.seq when Nav2 reported the arrival: a command that came
        in since then (stop, joystick, a new goal) has cancelled this already.
        """
        if seq != self.seq:
            return
        label = f'Masa {name}'
        if not self.store.settings['table_approach']:
            self._end()
            self.ros.event('info', f'Çatdı: {label}')
            return
        self.active = True
        with self.ros.lock:
            self.ros.nav = {'active': True, 'kind': 'approach', 'text': f'{label}: dəqiq yanaşma',
                            'current': 0, 'total': 1, 'remaining': None}
        outcome = None
        try:
            outcome = await self._approach(seq, label)
        except Exception as exc:  # noqa: B902 - report it; the robot is stopped below
            outcome = ('error', f'{label}: dəqiq yanaşma alınmadı ({exc})')
        finally:
            for _ in range(3):
                self.ros.drive(0.0, 0.0)
            if seq == self.seq:
                self._end()
        if outcome is not None:
            self.ros.event(*outcome)

    async def _approach(self, seq, label):
        """Do the approach: (level, text) to report, or None when cancelled."""
        await asyncio.sleep(self.settle)
        if seq != self.seq:
            return None
        table = self.table_ahead()
        if table is None:
            return 'info', f'Çatdı: {label} (masa kamerada görünmədi, dəqiq yanaşma edilmədi)'
        bearing = math.atan2(table[1], table[0])
        if abs(bearing) > 0.06:
            result = await self.turn_by(bearing, seq)
            if result != OK:
                return self._failed(result, label)
            await asyncio.sleep(self.pause)
            # Not seen after turning: the same point, now straight ahead.
            table = self.table_ahead() or (math.hypot(*table), 0.0)
        move = table[0] - self.robot_front() - self.store.settings['table_gap']
        if abs(move) > 0.03:
            result = await self.drive_by(max(-0.4, min(0.5, move)), seq)
            if result != OK:
                return self._failed(result, label)
            await asyncio.sleep(self.pause)
        if seq != self.seq:
            return None
        table = self.table_ahead()
        final = f'{(table[0] - self.robot_front()) * 100:.0f} sm' if table else '?'
        return 'info', f'Çatdı: {label} (masaya qədər {final})'

    @staticmethod
    def _failed(result, label):
        if result == CANCELLED:
            return None
        if result == NO_ODOM:
            return 'error', f'{label}: dəqiq yanaşma dayandırıldı (odometriya yoxdur)'
        return 'warn', (f'{label}: dəqiq yanaşma tamamlanmadı (yol bağlıdır?), '
                        'robot olduğu yerdə dayandı')

    # ---------------- sensing ----------------

    def table_ahead(self, max_age=0.8):
        """Get the nearest table the camera sees ahead: its near side (x, y) in base_footprint."""
        objs = self.ros.latest_objects(max_age=max_age)
        best = None
        for o in (objs or {}).get('objects', []):
            if o.get('cls') != 'table' or 'base' not in o:
                continue
            bx, by = o['base']
            if 0.1 < bx < 2.0 and abs(math.atan2(by, bx)) < 0.6:
                if best is None or bx < best[0]:
                    best = (bx, by)
        return best

    # ---------------- moves (odometry closed loop) ----------------

    async def turn_by(self, angle, seq):
        """Turn in place by `angle` (rad, + = left): OK, CANCELLED, TIMEOUT or NO_ODOM."""
        start = self.ros.odom_pose()
        if start is None:
            return NO_ODOM
        target = start['yaw'] + angle
        deadline = time.monotonic() + self.turn_timeout
        while time.monotonic() < deadline:
            if seq != self.seq:
                return CANCELLED
            pose = self.ros.odom_pose()
            if pose is None:
                return NO_ODOM
            err = wrap(target - pose['yaw'])
            if abs(err) < self.turn_tolerance:
                return OK
            # Below ~0.25 rad/s the loaded wheels stick; above 0.4 overshoots.
            self.ros.drive(0.0, math.copysign(min(0.4, max(0.25, 1.5 * abs(err))), err))
            await asyncio.sleep(self.step)
        return TIMEOUT

    def drive_timeout(self, distance):
        """Give the time allowed to drive `distance`: room for the people slowdown (half speed)."""
        return abs(distance) / self.speed * 2.5 + 2.0

    async def drive_by(self, distance, seq):
        """Drive straight by `distance` (m, + = forward): OK, CANCELLED, TIMEOUT or NO_ODOM."""
        start = self.ros.odom_pose()
        if start is None:
            return NO_ODOM
        deadline = time.monotonic() + self.drive_timeout(distance)
        while time.monotonic() < deadline:
            if seq != self.seq:
                return CANCELLED
            pose = self.ros.odom_pose()
            if pose is None:
                return NO_ODOM
            if math.hypot(pose['x'] - start['x'], pose['y'] - start['y']) >= abs(distance) - 0.01:
                return OK
            self.ros.drive(math.copysign(self.speed, distance), 0.0)
            await asyncio.sleep(self.step)
        return TIMEOUT
