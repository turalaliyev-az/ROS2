"""ROS interface of the web server: robot state in, commands out."""
import collections
import json
import math
import threading
import time

from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose, Spin
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rover_bringup import profile as robot_profile
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
SUCCEEDED, CANCELED = 4, 5
MANUAL_MAX_TURN = robot_profile.MAX_TURN_RATE  # rad/s

# Scans drawn on the phone's map: (topic, keep every n-th ray, max range m).
SCAN_SOURCES = {'lidar': ('scan', 2, 8.0), 'camera': ('camera_scan', 4, 4.0)}
# Camera pictures are only subscribed while someone is looking at them: the
# raw colour stream alone is ~27 MB/s of DDS traffic.
CAMERA_TOPICS = {'color': '/camera/camera/color/image_raw',
                 'depth': '/camera/camera/depth/image_rect_raw'}
CAMERA_IDLE_S = 5.0


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_pose(x, y, yaw, stamp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class RosSide(Node):

    def __init__(self):
        super().__init__('rover_web_server')
        self.declare_parameter('port', 8080)
        # Mode entered at boot: navigation | mapping | idle | none (none: another
        # process runs Nav2, e.g. the loopback simulator).
        self.declare_parameter('start_mode', 'navigation')

        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=50)
        self.map_msg = None
        self.map_version = 0
        self.amcl = None
        self.amcl_count = 0
        self.last_seen = {'esp32': 0.0, 'lidar': 0.0, 'camera': 0.0}
        self.velocity = (0.0, 0.0)
        self.explore_status = ''
        self.nav = {'active': False, 'kind': '', 'text': '', 'current': 0, 'total': 0,
                    'remaining': None}
        self.nav_speed = 0.2
        # Nav2's turn regulation follows nav_speed (see _tune_controller).
        self.controller_params = AsyncParameterClient(self, 'controller_server')
        self.tuned_speed = None  # nav_speed the controller is tuned for
        self._tuning = False
        self.scans = {}          # 'lidar' | 'camera' -> (receive time, latest LaserScan)
        self.lidar_recent = collections.deque(maxlen=10)  # (receive time, LaserScan)
        self.camera_msgs = {}    # 'color' | 'depth' -> latest Image
        self.camera_wanted = {}  # kind -> monotonic time of the last request
        self.camera_subs = {}
        self.objects = None          # latest /perception/objects (parsed JSON)
        self.objects_time = 0.0
        self.perception_status = ''
        self.monitor_state = (0, '')  # collision monitor (action_type, polygon_name)
        self.on_table_arrived = None  # Controller hook: called with the table's name
        self.drive_fault = None       # latest motor_bridge drive fault (dict) while it lasts

        self.goal_handle = None
        self.goal_seq = 0
        self._spins_left = 0
        self.route_points = []  # [(x, y)] of the route being driven

        self.create_subscription(OccupancyGrid, 'map', self._on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, LATCHED)
        self.create_subscription(Odometry, 'wheel_odom', self._on_odom, 10)
        for key, (topic, _, _) in SCAN_SOURCES.items():
            self.create_subscription(LaserScan, topic, self._scan_callback(key),
                                     qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info',
                                 self._seen('camera'), qos_profile_sensor_data)
        self.create_subscription(String, 'explore/status', self._on_explore, LATCHED)
        self.create_subscription(String, 'perception/objects', self._on_objects, 10)
        self.create_subscription(String, 'perception/status', self._on_perception_status, LATCHED)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self._on_monitor_state, 10)
        self.create_subscription(String, 'drive/fault', self._on_drive_fault, LATCHED)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # collision_monitor's input: manual driving through here gets the same
        # "slow down / stop before an obstacle" protection as Nav2.
        self.safe_cmd_pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.people_guard_pub = self.create_publisher(Bool, 'perception/people_guard', LATCHED)
        # Where the robot is heading, for the detector: people there are the
        # customers it is going to, not obstacles. Empty frame_id = none.
        self.destination_pub = self.create_publisher(PoseStamped, 'perception/destination',
                                                     LATCHED)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.global_loc = self.create_client(Empty, 'reinitialize_global_localization')
        # Direct cancel services, to stop goals this node didn't send (explorer, CLI).
        self.cancel_all_clients = [
            self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
            for action in ('navigate_to_pose', 'follow_waypoints', 'spin')]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Resent every second: controller_server comes and goes with the mode.
        self.create_timer(1.0, self._publish_speed_limit)
        self.create_timer(1.0, self._manage_camera_subs)

    # ---------------- state in ----------------

    def event(self, level, text):
        self.events.append((level, text))

    def _seen(self, key):
        def callback(_msg):
            self.last_seen[key] = time.monotonic()
        return callback

    def _scan_callback(self, key):
        def callback(msg):
            # Receive time, not header stamps: the camera's may run on its own clock.
            self.scans[key] = (time.monotonic(), msg)
            if key == 'lidar':
                self.last_seen['lidar'] = time.monotonic()
                self.lidar_recent.append(self.scans[key])
        return callback

    def lidar_points(self, since):
        """
        Merge the lidar scans received after `since` (monotonic) into robot-frame points.

        Returns (Nx2 array, number of scans), or None.
        """
        scans = [m for t, m in list(self.lidar_recent) if t > since]
        if not scans:
            return None
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', scans[0].header.frame_id,
                                                rclpy.time.Time())
        except TransformException:
            return None
        rot, tr = quat_matrix(t.transform.rotation), t.transform.translation
        pts = []
        for msg in scans:
            r = np.asarray(msg.ranges, dtype=float)
            a = msg.angle_min + msg.angle_increment * np.arange(len(r))
            ok = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (r < min(msg.range_max, 15.0))
            local = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])
            world = rot @ local
            pts.append(np.stack([world[0] + tr.x, world[1] + tr.y], axis=1))
        return np.concatenate(pts), len(scans)

    def scan_points(self, frame):
        """Recent scans as flat [x0, y0, x1, y1, ...] lists in `frame`, for drawing."""
        out = {}
        now = time.monotonic()
        for key, (_, step, max_range) in SCAN_SOURCES.items():
            received, msg = self.scans.get(key, (0.0, None))
            if msg is None or now - received > 1.0:
                continue
            try:
                t = self.tf_buffer.lookup_transform(frame, msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                continue
            ranges = np.asarray(msg.ranges, dtype=np.float32)[::step]
            angles = msg.angle_min + np.arange(len(ranges)) * step * msg.angle_increment
            ok = np.isfinite(ranges) & (ranges > max(msg.range_min, 0.05)) & \
                (ranges < min(msg.range_max, max_range))
            r, a = ranges[ok], angles[ok]
            local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
            world = quat_matrix(t.transform.rotation) @ local
            tr = t.transform.translation
            xy = np.stack([world[0] + tr.x, world[1] + tr.y], axis=1)
            out[key] = np.round(xy, 2).ravel().tolist()
        return out

    # ---------------- camera pictures (on demand) ----------------

    def want_camera(self, kind):
        self.camera_wanted[kind] = time.monotonic()

    def _manage_camera_subs(self):
        now = time.monotonic()
        for kind, topic in CAMERA_TOPICS.items():
            wanted = now - self.camera_wanted.get(kind, -1e9) < CAMERA_IDLE_S
            if wanted and kind not in self.camera_subs:
                self.camera_subs[kind] = self.create_subscription(
                    Image, topic, self._image_callback(kind), qos_profile_sensor_data)
            elif not wanted and kind in self.camera_subs:
                self.destroy_subscription(self.camera_subs.pop(kind))
                self.camera_msgs.pop(kind, None)

    def _image_callback(self, kind):
        def callback(msg):
            self.camera_msgs[kind] = msg
        return callback

    def _on_map(self, msg):
        with self.lock:
            self.map_msg = msg
            self.map_version += 1

    def _on_amcl(self, msg):
        cov = msg.pose.covariance
        p = msg.pose.pose
        with self.lock:
            self.amcl = {
                'x': p.position.x, 'y': p.position.y, 'yaw': quat_to_yaw(p.orientation),
                'std_xy': math.sqrt(max(cov[0], cov[7], 0.0)),
                'std_yaw': math.sqrt(max(cov[35], 0.0)),
            }
            self.amcl_count += 1

    def _on_odom(self, msg):
        self.last_seen['esp32'] = time.monotonic()
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_explore(self, msg):
        self.explore_status = msg.data

    def _on_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.objects = data
            self.objects_time = time.monotonic()

    def _on_perception_status(self, msg):
        self.perception_status = msg.data

    def _on_monitor_state(self, msg):
        # Published when the collision monitor's action changes.
        self.monitor_state = (msg.action_type, msg.polygon_name)

    def _on_drive_fault(self, msg):
        # motor_bridge has already stopped the motors; make sure nothing keeps
        # commanding them and tell the operator what to look at.
        if not msg.data:
            if self.drive_fault is not None:
                self.event('info', 'Ötürücü nasazlığı aradan qalxdı, robot yenidən '
                                   'hərəkət edə bilər')
            self.drive_fault = None
            return
        try:
            fault = json.loads(msg.data)
        except ValueError:
            return
        if self.drive_fault is not None and self.drive_fault.get('time') == fault.get('time'):
            return
        self.drive_fault = fault
        side = {'left': 'Sol', 'right': 'Sağ'}.get(fault.get('wheel'), 'Bir')
        self.cancel_everything()
        target, measured = abs(fault.get('target', 0)), abs(fault.get('measured', 0))
        self.event('error', f'{side} təkər hərəkət etmir (əmr {target:.2f} m/s, faktiki '
                            f'{measured:.2f} m/s): robot dayandırıldı. Maneəni, təkəri və '
                            'mühərrik kabelini yoxlayın.')

    def latest_objects(self, max_age=1.5):
        with self.lock:
            if self.objects is None or time.monotonic() - self.objects_time > max_age:
                return None
            return self.objects

    def set_people_guard(self, on):
        """Turn the people zones of the collision monitor on or off (via the detector)."""
        self.people_guard_pub.publish(Bool(data=bool(on)))

    def set_destination(self, x, y):
        """Tell the detector where the robot is heading (map frame)."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.destination_pub.publish(msg)

    def clear_destination(self):
        """Tell the detector the robot is heading nowhere: nobody is exempt."""
        self.destination_pub.publish(PoseStamped())

    def odom_pose(self):
        """Robot pose in odom: smooth, for short closed-loop moves."""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        tr = t.transform
        return {'x': tr.translation.x, 'y': tr.translation.y, 'yaw': quat_to_yaw(tr.rotation)}

    def sensors(self):
        now = time.monotonic()
        return {k: (now - t) < 1.5 for k, t in self.last_seen.items()}

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        tr = t.transform
        return {'x': tr.translation.x, 'y': tr.translation.y, 'yaw': quat_to_yaw(tr.rotation)}

    # ---------------- manual driving / speed ----------------

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        if self.count_subscribers('cmd_vel_smoothed') > 0:
            self.safe_cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(msg)

    def _publish_speed_limit(self):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = float(self.nav_speed)
        self.speed_pub.publish(msg)
        self._tune_controller()

    def _tune_controller(self):
        """
        Keep Regulated Pure Pursuit's turn regulation in step with the speed slider.

        The params file holds the value for the top speed, so a controller
        that was never tuned is careful rather than fast in turns. Retried
        every second until it takes, and again whenever the controller comes
        back (it restarts with the mode).
        """
        if not self.controller_params.services_are_ready():
            self.tuned_speed = None
            return
        speed = self.nav_speed
        if self._tuning or self.tuned_speed == speed:
            return
        self._tuning = True
        future = self.controller_params.set_parameters([Parameter(
            'FollowPath.regulated_linear_scaling_min_radius',
            value=float(robot_profile.min_turn_radius(speed)))])
        future.add_done_callback(lambda f: self._on_tuned(speed, f))

    def _on_tuned(self, speed, future):
        self._tuning = False
        try:
            results = future.result().results
        except Exception:  # noqa: B902 - service gone mid-call: retried next second
            results = []
        if results and all(r.successful for r in results):
            self.tuned_speed = speed

    # ---------------- localization ----------------

    def set_initial_pose(self, x, y, yaw, std_xy=0.3, std_yaw=0.25):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = std_xy ** 2
        msg.pose.covariance[7] = std_xy ** 2
        msg.pose.covariance[35] = std_yaw ** 2
        self.initialpose_pub.publish(msg)

    def relocalize(self):
        """Scatter AMCL's particles over the whole map, then turn twice in place to settle."""
        if not self.global_loc.service_is_ready() or not self.spin_client.server_is_ready():
            self.event('error', 'Lokallaşma hazır deyil (naviqasiya rejimi işləyirmi?)')
            return
        self.cancel()
        self.global_loc.call_async(Empty.Request())
        self._spins_left = 2
        self._send_spin()

    def _send_spin(self):
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = Duration(sec=40)
        self._send(self.spin_client, goal, 'relocalize', 'Mövqe axtarılır (yerində dönür)',
                   total=2)

    # ---------------- goals ----------------

    def goal_yaw(self, x, y):
        pose = self.robot_pose()
        if pose is None:
            return 0.0
        return math.atan2(y - pose['y'], x - pose['x'])

    def goto(self, x, y, yaw, label, kind='goto'):
        """kind: 'goto', or 'home' for a trip to the charging spot."""
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw, self.get_clock().now().to_msg())
        sent = self._send(self.nav_client, goal, kind, label, total=1)
        if sent:
            self.set_destination(x, y)
        return sent

    def route(self, points, loops, label):
        """
        Drive through `points`: [(x, y, yaw or None)].

        A missing heading means arrive facing the direction of travel.
        """
        stamp = self.get_clock().now().to_msg()
        pose = self.robot_pose()
        prev = (pose['x'], pose['y']) if pose else None
        poses = []
        for x, y, yaw in points:
            if yaw is None:
                yaw = math.atan2(y - prev[1], x - prev[0]) if prev and (x, y) != prev else 0.0
            poses.append(make_pose(x, y, yaw, stamp))
            prev = (x, y)
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        goal.number_of_loops = int(loops)
        sent = self._send(self.route_client, goal, 'route', label, total=len(points))
        if sent:
            self.route_points = [(x, y) for x, y, _ in points]
            self.set_destination(*self.route_points[0])
        return sent

    def _send(self, client, goal, kind, label, total):
        if not client.server_is_ready():
            self.event('error', 'Naviqasiya hazır deyil')
            return False
        if kind != 'relocalize' or self._spins_left == 2:
            self._drop_goal()
        self.goal_seq += 1
        seq = self.goal_seq
        with self.lock:
            self.nav = {'active': True, 'kind': kind, 'text': label, 'current': 0,
                        'total': total, 'remaining': None}
        future = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(seq, kind, fb))
        future.add_done_callback(lambda f: self._on_goal_response(seq, f))
        return True

    def _on_goal_response(self, seq, future):
        handle = future.result()
        if seq != self.goal_seq:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            with self.lock:
                self.nav['active'] = False
            self.event('error', 'Hədəf qəbul edilmədi')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_feedback(self, seq, kind, fb):
        if seq != self.goal_seq:
            return
        next_point = None
        with self.lock:
            if kind in ('goto', 'home', 'table'):
                self.nav['remaining'] = round(fb.feedback.distance_remaining, 2)
            elif kind == 'route':
                current = fb.feedback.current_waypoint
                if current != self.nav['current'] and current < len(self.route_points):
                    next_point = self.route_points[current]
                self.nav['current'] = current
        if next_point is not None:
            self.set_destination(*next_point)

    def _on_result(self, seq, future):
        if seq != self.goal_seq:
            return
        wrapped = future.result()
        status, result = wrapped.status, wrapped.result
        self.goal_handle = None
        with self.lock:
            kind, label = self.nav['kind'], self.nav['text']
            self.nav['active'] = False
        # A table reached hands over to the camera-guided approach, which still
        # needs the customers there left out; it clears the destination itself.
        if not (kind == 'table' and status == SUCCEEDED and self.on_table_arrived is not None):
            self.clear_destination()

        if kind == 'relocalize':
            self._spins_left -= 1
            if status == SUCCEEDED and self._spins_left > 0:
                self._send_spin()
                with self.lock:
                    self.nav['current'] = 1
            elif status == SUCCEEDED:
                self.event('info', 'Mövqe axtarışı bitdi')
            elif status != CANCELED:
                self.event('error', 'Yerində dönmək mümkün olmadı (yaxında maneə var?)')
            return

        if status == SUCCEEDED:
            missed = [m.index + 1 for m in getattr(result, 'missed_waypoints', [])]
            if missed:
                self.event('warn', f'{label}: bu nöqtələrə çatmaq olmadı: {missed}')
            elif kind == 'home':
                self.event('info', 'Robot evə çatdı')
            elif kind == 'table' and self.on_table_arrived is not None:
                self.on_table_arrived(label)
            else:
                self.event('info', f'Çatdı: {label}')
        elif status == CANCELED:
            self.event('info', f'Ləğv edildi: {label}')
        else:
            self.event('error', f'Çatmaq mümkün olmadı: {label}')

    def _drop_goal(self):
        self.goal_seq += 1
        handle, self.goal_handle = self.goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def cancel(self):
        """Cancel the goal this node sent; the robot heads nowhere."""
        self._spins_left = 0
        self._drop_goal()
        with self.lock:
            self.nav['active'] = False
        self.clear_destination()

    def cancel_everything(self):
        """Cancel every navigation goal, whoever sent it."""
        self.cancel()
        for client in self.cancel_all_clients:
            if client.service_is_ready():
                # Zero goal id and zero stamp: "all goals" in the action protocol.
                client.call_async(CancelGoal.Request())

    def nav_active(self):
        with self.lock:
            return self.nav['active']
