"""
Is the robot localized? Put the current lidar scan on the map at the pose the
robot believes it has and count how many points land on mapped walls.
Rover service running in navigation mode. Usage: loc_check.py
"""
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
rclpy.init()
node = rclpy.create_node('loc_check')
buf = Buffer()
TransformListener(buf, node)
got = {}
node.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('map', m), LATCHED)
node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: got.__setitem__('amcl', m), LATCHED)
node.create_subscription(LaserScan, 'scan', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
end = time.monotonic() + 15
while time.monotonic() < end and not {'map', 'scan'} <= got.keys():
    rclpy.spin_once(node, timeout_sec=0.1)
if 'map' not in got:
    raise SystemExit('no map (is navigation mode running?)')
t = None
end = time.monotonic() + 5
while t is None and time.monotonic() < end:   # let the TF buffer fill
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        t = buf.lookup_transform('map', got['scan'].header.frame_id, rclpy.time.Time())
    except Exception:  # noqa: B902
        t = None
if t is None:
    raise SystemExit('no map -> lidar transform (is AMCL running?)')
scan = got['scan']
m = got['map']
grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
occupied = grid >= 65
# distance (m) from every cell to the nearest mapped wall
dist = ndimage.distance_transform_edt(~occupied) * m.info.resolution
q, tr = t.transform.rotation, t.transform.translation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
r = np.asarray(scan.ranges, dtype=float)
a = scan.angle_min + scan.angle_increment * np.arange(len(r))
ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, 10.0))
px = tr.x + r[ok] * np.cos(a[ok] + yaw)
py = tr.y + r[ok] * np.sin(a[ok] + yaw)
i = np.floor((px - m.info.origin.position.x) / m.info.resolution).astype(int)
j = np.floor((py - m.info.origin.position.y) / m.info.resolution).astype(int)
inside = (i >= 0) & (i < m.info.width) & (j >= 0) & (j < m.info.height)
d = dist[j[inside], i[inside]]
hit = float(np.mean(d < 0.10)) if len(d) else 0.0
print(f'robotun güman etdiyi yer: x {tr.x:+.2f}, y {tr.y:+.2f}, bucaq {math.degrees(yaw):+.0f}°')
if 'amcl' in got:
    c = got['amcl'].pose.covariance
    print(f'AMCL qeyri-müəyyənliyi: ±{math.sqrt(max(c[0], c[7])):.2f} m, ±{math.degrees(math.sqrt(c[35])):.0f}°')
print(f'skan nöqtələri: {ok.sum()}, xəritədə {inside.sum()}; divara 10 sm-dən yaxın: {100 * hit:.0f}% '
      f'(median məsafə {np.median(d) * 100 if len(d) else float("nan"):.0f} sm)')
print('NƏTİCƏ: ' + ('LOKALİZASİYA YAXŞIDIR' if hit > 0.6 else
                   'ŞÜBHƏLİDİR — mövqeyi yoxlayın' if hit > 0.35 else
                   'LOKALİZASİYA YOXDUR — veb paneldə robotun mövqeyini xəritədə təyin edin'))
node.destroy_node()
rclpy.shutdown()
