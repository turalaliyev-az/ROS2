"""
Find where the robot is on the map from its lidar scan alone (correlative scan
matching, rover_web.scan_match: the same search the web server runs when
navigation starts and on "Mövqeyi tap"), and give AMCL that pose. Rover
service running in navigation mode, robot standing still.

The pose is only sent when it is clearly better than any other place (no
look-alike rooms).
Usage: global_localize.py [--dry-run]
"""
import math
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rover_web import scan_match
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
CLEARANCE = 0.2     # m: the robot's centre can't be closer than this to a wall

rclpy.init()
node = rclpy.create_node('global_localize')
buf = Buffer()
TransformListener(buf, node)
pose_pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
got = {'scans': []}
node.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('map', m), LATCHED)
node.create_subscription(LaserScan, 'scan',
                         lambda m: got['scans'].append(m) if len(got['scans']) < 10 else None,
                         qos_profile_sensor_data)
end = time.monotonic() + 15
while time.monotonic() < end and ('map' not in got or len(got['scans']) < 10):
    rclpy.spin_once(node, timeout_sec=0.1)
if 'map' not in got or not got['scans']:
    raise SystemExit('no map or scan (navigation mode running?)')
laser = None
end = time.monotonic() + 5
while laser is None and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        laser = buf.lookup_transform('base_footprint', got['scans'][0].header.frame_id, rclpy.time.Time())
    except Exception:  # noqa: B902
        laser = None
if laser is None:
    raise SystemExit('no base_footprint -> lidar transform')

# Scan points in the robot frame, several scans merged (beams vary per scan).
lq, lt = laser.transform.rotation, laser.transform.translation
lyaw = math.atan2(2 * (lq.w * lq.z + lq.x * lq.y), 1 - 2 * (lq.y * lq.y + lq.z * lq.z))
pts = []
for s in got['scans']:
    r = np.asarray(s.ranges, dtype=float)
    a = s.angle_min + s.angle_increment * np.arange(len(r)) + lyaw
    ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 15.0))
    pts.append(np.stack([lt.x + r[ok] * np.cos(a[ok]), lt.y + r[ok] * np.sin(a[ok])], axis=1))
pts = scan_match.thin(np.concatenate(pts))

m = got['map']
field = scan_match.MapField(m.info.resolution, m.info.width, m.info.height,
                            m.info.origin.position.x, m.info.origin.position.y, m.data)
print(f'skan: {len(pts)} nöqtə; xəritə {m.info.width} × {m.info.height}', flush=True)
t0 = time.monotonic()
res = scan_match.localize(field, pts, CLEARANCE)
if res is None:
    raise SystemExit('xəritədə robot sığan boş yer yoxdur')
x, y, th = res['x'], res['y'], res['yaw']
print(f'axtarış: {time.monotonic() - t0:.1f} s')
print(f'NƏTİCƏ: ən yaxşı x {x:+.2f}, y {y:+.2f}, bucaq {math.degrees(th):+.0f}°; '
      f'divara düşən {100 * res["hits"]:.0f}%, ikincidən fərq {res["margin"]:.3f} → '
      + ('ETİBARLI' if res['trusted'] else 'QEYRİ-MÜƏYYƏN (göndərilmədi)'))
if res['trusted'] and '--dry-run' not in sys.argv:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.1 ** 2
    msg.pose.covariance[35] = math.radians(5) ** 2
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pose_pub.publish(msg)
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    print('mövqe AMCL-ə göndərildi (/initialpose)')
node.destroy_node()
rclpy.shutdown()
