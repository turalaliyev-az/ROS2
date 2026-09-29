"""
Frontier exploration for autonomous mapping.

Runs next to slam_toolbox and Nav2: repeatedly sends the robot to the nearest
worthwhile boundary between mapped free space and unknown space, until no such
boundary is left, then drives back to where it started. Progress is reported
as a string on /explore/status (latched).
"""
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        self.declare_parameter('min_frontier_size_m', 0.4)
        # A goal closer than this to a wall puts the ~0.31 m (circumscribed)
        # footprint into lethal cost, and the planner rejects it.
        self.declare_parameter('robot_clearance_m', 0.35)
        self.declare_parameter('goal_timeout_s', 90.0)
        self.declare_parameter('blacklist_radius_m', 0.6)
        # Unknown pockets smaller than this inside mapped space (the cells under
        # the lidar itself, shadows behind chair legs) are not openings.
        self.declare_parameter('min_unknown_area_m2', 0.25)
        # Frontiers this close are being looked at already; driving "there" is
        # a no-op that never moves the robot.
        self.declare_parameter('min_goal_distance_m', 0.6)
        self.declare_parameter('return_home', True)

        self.min_size = self.get_parameter('min_frontier_size_m').value
        self.clearance = self.get_parameter('robot_clearance_m').value
        self.goal_timeout = self.get_parameter('goal_timeout_s').value
        self.blacklist_radius = self.get_parameter('blacklist_radius_m').value
        self.min_unknown_area = self.get_parameter('min_unknown_area_m2').value
        self.min_goal_distance = self.get_parameter('min_goal_distance_m').value
        self.return_home = self.get_parameter('return_home').value

        self.map = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, LATCHED)
        self.status_pub = self.create_publisher(String, 'explore/status', LATCHED)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.home = None
        self.started = 0.0
        self.looked_around = False
        self.spin_handle = None
        self.spinning = False
        self.goal_handle = None
        self.goal_xy = None
        self.goal_started = 0.0
        self.goal_pending = False
        self.cancel_reason = None  # why *we* cancelled the current goal
        self.returning = False
        self.finished = False
        self.blacklist = []
        self.reached = []  # frontier goals the robot got to
        self.empty_checks = 0

        self._set_status('starting')
        self.create_timer(1.0, self._tick)

    # ---------------- helpers ----------------

    def _set_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'explore: {text}')

    def _on_map(self, msg):
        self.map = msg

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _inside_map(self, x, y):
        info = self.map.info
        col = (x - info.origin.position.x) / info.resolution
        row = (y - info.origin.position.y) / info.resolution
        return 0 <= col < info.width and 0 <= row < info.height

    def _blacklisted(self, x, y):
        if any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist):
            return True
        # Reached twice and still open: nothing more can be seen from there.
        return sum(math.hypot(x - rx, y - ry) < 0.5 for rx, ry in self.reached) >= 2

    def _frontiers(self):
        """Return candidate goals [(x, y, size_m)] from the current map."""
        m = self.map
        res = m.info.resolution
        grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        free = grid == 0
        occupied = grid >= 50
        # Everything beyond the grid's edge is unexplored too: free space that
        # runs up to the edge is a frontier. (One-cell unknown border.)
        unknown = np.pad(grid < 0, 1, constant_values=True)

        pockets, count = ndimage.label(unknown)
        if count:
            area = ndimage.sum(unknown, pockets, index=np.arange(1, count + 1)) * res * res
            keep = np.concatenate([[False], area >= self.min_unknown_area])
            unknown = keep[pockets]

        next_to_unknown = (unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
                           unknown[1:-1, :-2] | unknown[1:-1, 2:])
        frontier = free & next_to_unknown

        # A frontier squeezed against a wall is a gap the robot can't reach
        # (or a sensor shadow), not a real opening.
        if occupied.any():
            wall_dist = ndimage.distance_transform_edt(~occupied) * res
            frontier &= wall_dist > self.clearance

        labels, count = ndimage.label(frontier, structure=np.ones((3, 3)))
        goals = []
        for idx in range(1, count + 1):
            rows, cols = np.nonzero(labels == idx)
            size_m = len(rows) * res
            if size_m < self.min_size:
                continue
            # The cluster cell nearest its centroid: always on the frontier,
            # unlike the centroid itself for a curved frontier.
            cr, cc = rows.mean(), cols.mean()
            k = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
            x = m.info.origin.position.x + (cols[k] + 0.5) * res
            y = m.info.origin.position.y + (rows[k] + 0.5) * res
            goals.append((x, y, size_m))
        return goals

    # ---------------- navigation ----------------

    def _send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_xy = (x, y)
        self.goal_started = self._now()
        self.goal_pending = True
        self.cancel_reason = None
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_pending = False
            self._goal_failed()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.goal_handle = None
        self.goal_pending = False
        if self.returning:
            self.finished = True
            self._set_status('done')
            return
        reason, self.cancel_reason = self.cancel_reason, None
        if status == 4:  # SUCCEEDED
            self.reached.append(self.goal_xy)
            return
        # A frontier we dropped because it got mapped on the way is not a
        # failure: blacklisting it would also hide real frontiers next to it.
        if status == 5 and reason == 'uncovered':
            return
        self._goal_failed()

    def _goal_failed(self):
        if self.goal_xy is not None and not self.returning:
            self.blacklist.append(self.goal_xy)
            self.get_logger().info(f'frontier at ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f}) '
                                   'unreachable, skipping it')

    def _cancel_goal(self, reason):
        self.cancel_reason = reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()

    def _look_around(self):
        """One slow turn in place before picking any goal.

        slam_toolbox adds a scan only after the robot has moved (0.2 m/0.2 rad)
        and marks a cell only once two scans have seen it, so a robot that has
        not moved yet has hardly any map to find frontiers in.
        """
        if not self.spin.server_is_ready():
            self.looked_around = True
            return
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = Duration(sec=60)
        self.spinning = True
        self._set_status('looking around')
        self.spin.send_goal_async(goal).add_done_callback(self._on_spin_response)

    def _on_spin_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._on_spin_done(None)
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(self._on_spin_done)

    def _on_spin_done(self, _future):
        self.spin_handle = None
        self.spinning = False
        self.looked_around = True
        self.started = self._now()  # the "nothing left" grace period starts now
        self._set_status('exploring')

    # ---------------- main loop ----------------

    def _tick(self):
        if self.finished or self.map is None:
            return
        robot = self._robot_xy()
        if robot is None:
            return
        if not self.nav.server_is_ready():
            self._set_status('waiting for navigation')
            return
        if self.home is None:
            self.home = robot
            self.started = self._now()
        if not self.looked_around:
            if not self.spinning:
                self._look_around()
            return

        if self.goal_pending or self.goal_handle is not None:
            if self.returning:
                return
            if self._now() - self.goal_started > self.goal_timeout:
                if self.cancel_reason is None:
                    self.get_logger().info('frontier goal timed out')
                    self._cancel_goal('timeout')  # blacklisted when the result arrives
                return
            # Frontier already uncovered on the way there: pick the next one
            # instead of driving to a spot that no longer needs a look.
            still_open = any(math.hypot(gx - self.goal_xy[0], gy - self.goal_xy[1]) < 0.5
                             for gx, gy, _ in self._frontiers())
            if not still_open and self.goal_handle is not None and self.cancel_reason is None:
                self._cancel_goal('uncovered')
            return

        if not self._inside_map(*robot):
            return  # the planner can't start from outside the map; wait for slam to grow it
        candidates = [(x, y, s) for x, y, s in self._frontiers()
                      if not self._blacklisted(x, y) and
                      math.hypot(x - robot[0], y - robot[1]) >= self.min_goal_distance]
        if not candidates:
            # slam_toolbox updates the map every few seconds, and its first
            # maps may not even cover the robot yet: make sure the frontiers
            # really are gone before calling it finished.
            self.empty_checks += 1
            if self.empty_checks < 10 or self._now() - self.started < 30.0 or \
                    not self._inside_map(*robot):
                return
            if self.return_home and self.home is not None:
                self.returning = True
                self._set_status('returning home')
                self._send_goal(self.home[0], self.home[1],
                                math.atan2(self.home[1] - robot[1], self.home[0] - robot[0]))
            else:
                self.finished = True
                self._set_status('done')
            return
        self.empty_checks = 0

        # Nearest first, with big openings worth a bit of a detour.
        x, y, _ = min(candidates, key=lambda c: math.hypot(c[0] - robot[0], c[1] - robot[1])
                      / (1.0 + c[2]))
        self._set_status(f'exploring ({len(candidates)} frontiers left)')
        self._send_goal(x, y, math.atan2(y - robot[1], x - robot[0]))


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_goal('shutdown')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
