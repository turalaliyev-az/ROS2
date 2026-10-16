"""
Watch navigation goals and report each one (read-only; rover service running).

Per goal: where it was going, result, time, path length driven, how close it
ended to the goal, top speed, recoveries, collision monitor actions, drive
faults, localization uncertainty. Lines are printed as they happen.
Usage: nav_monitor.py [minutes]
"""
import math
import sys
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry, Path
import rclpy
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MINUTES = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
STATUS = {1: 'qəbul edildi', 2: 'icra olunur', 3: 'ləğv olunur', 4: 'UĞURLU', 5: 'LƏĞV EDİLDİ', 6: 'UĞURSUZ'}
rclpy.init()
node = rclpy.create_node('nav_monitor')
goals = {}          # goal id -> dict
st = {'pose': None, 'std': None, 'last_odom': None}


def out(text):
    print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)


def gid(uuid):
    return bytes(uuid.uuid).hex()[:8]


def on_status(msg):
    for s in msg.status_list:
        k = gid(s.goal_info.goal_id)
        g = goals.setdefault(k, {'status': None, 'start': time.monotonic(), 'path': 0.0, 'vmax': 0.0,
                                 'recoveries': 0, 'cm': set(), 'faults': 0, 'goal': None})
        if s.status != g['status']:
            g['status'] = s.status
            if s.status == 2:
                out(f'hədəf {k}: icra olunur')
            elif s.status in (4, 5, 6):
                report(k, g)


def report(k, g):
    dt = time.monotonic() - g['start']
    err = ''
    if g['goal'] and st['pose']:
        gx, gy = g['goal']
        px, py, pyaw = st['pose']
        err = f", hədəfə qalan {math.hypot(gx - px, gy - py) * 100:.0f} sm"
    out(f"hədəf {k}: {STATUS.get(g['status'])} — {dt:.0f} s, yol {g['path']:.1f} m{err}, "
        f"maks sürət {g['vmax']:.2f} m/s, bərpa {g['recoveries']}, "
        f"collision monitor: {', '.join(sorted(g['cm'])) or 'yox'}, ötürücü nasazlığı {g['faults']}, "
        f"lokalizasiya ±{st['std'] or 0:.2f} m")


def active():
    return [g for g in goals.values() if g['status'] in (1, 2)]


def on_feedback(msg):
    k = gid(msg.goal_id)
    g = goals.get(k)
    if g is None:
        return
    g['recoveries'] = max(g['recoveries'], msg.feedback.number_of_recoveries)


def on_plan(msg):
    if msg.poses:
        p = msg.poses[-1].pose.position
        for g in active():
            if g['goal'] is None:
                out(f'  hədəf nöqtəsi: ({p.x:.2f}, {p.y:.2f}), plan uzunluğu '
                    f'{sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y) for a, b in zip(msg.poses, msg.poses[1:])):.1f} m')
            g['goal'] = (p.x, p.y)


def on_amcl(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    st['pose'] = (p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
    c = msg.pose.covariance
    st['std'] = math.sqrt(max(c[0], c[7]))


def on_odom(msg):
    v = abs(msg.twist.twist.linear.x)
    p = msg.pose.pose.position
    for g in active():
        g['vmax'] = max(g['vmax'], v)
        if st['last_odom'] is not None:
            g['path'] += math.hypot(p.x - st['last_odom'][0], p.y - st['last_odom'][1])
    st['last_odom'] = (p.x, p.y)


def on_cm(msg):
    if msg.action_type != 0:
        names = {1: 'dayan', 2: 'yavaşla', 3: 'yaxınlaşma', 4: 'limit'}
        for g in active():
            g['cm'].add(f"{msg.polygon_name}/{names.get(msg.action_type, msg.action_type)}")
        out(f'  collision monitor: {msg.polygon_name} ({names.get(msg.action_type, msg.action_type)})')


def on_fault(msg):
    if msg.data:
        for g in active():
            g['faults'] += 1
        out(f'  ⚠ ÖTÜRÜCÜ NASAZLIĞI: {msg.data}')


def on_log(msg):
    if msg.level >= 30 and msg.name in ('bt_navigator', 'controller_server', 'behavior_server',
                                        'planner_server', 'collision_monitor', 'motor_bridge_node'):
        if 'missed its desired rate' in msg.msg:
            return
        out(f'  [{msg.name}] {msg.msg[:140]}')


node.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', on_status, 10)
node.create_subscription(NavigateToPose.Impl.FeedbackMessage, 'navigate_to_pose/_action/feedback',
                         on_feedback, 10)
node.create_subscription(Path, 'plan', on_plan, 10)
node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_amcl, LATCHED)
node.create_subscription(Odometry, 'odom', on_odom, 10)
node.create_subscription(CollisionMonitorState, 'collision_monitor_state', on_cm, 10)
node.create_subscription(String, 'drive/fault', on_fault, LATCHED)
node.create_subscription(Log, 'rosout', on_log, 50)
out(f'monitor işləyir ({MINUTES:.0f} dəq): hədəfləri veb paneldən verin')
end = time.monotonic() + MINUTES * 60
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
