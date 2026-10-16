"""
Check the depth camera's pose against the lidar (rover service running, robot
standing still in front of walls / furniture).

Both the lidar scan and the camera's depth-derived scan (camera_scan) are put
into base_footprint with the current TF, then the camera points are aligned to
the lidar points with 2D ICP. The resulting shift is the error of the camera
pose in the robot profile (walls look the same to both sensors).
Usage: camera_check.py
"""
import math
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('camera_check')
buf = Buffer()
TransformListener(buf, node)
got = {'scan': [], 'camera_scan': []}


def collect(name):
    def cb(msg):
        if len(got[name]) < 15:
            got[name].append(msg)
    return cb


node.create_subscription(LaserScan, 'scan', collect('scan'), qos_profile_sensor_data)
node.create_subscription(LaserScan, 'camera_scan', collect('camera_scan'), qos_profile_sensor_data)
end = time.monotonic() + 15
while time.monotonic() < end and (len(got['scan']) < 15 or len(got['camera_scan']) < 15):
    rclpy.spin_once(node, timeout_sec=0.1)


def to_base(msgs, max_range):
    pts = []
    for m in msgs:
        try:
            t = buf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
        except Exception:  # noqa: B902
            continue
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, max_range))
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s = math.cos(yaw), math.sin(yaw)
        pts.append(np.stack([c * x - s * y + tr.x, s * x + c * y + tr.y], axis=1))
    return np.concatenate(pts) if pts else np.zeros((0, 2))


lidar = to_base(got['scan'], 6.0)
cam = to_base(got['camera_scan'], 4.0)
if len(lidar) < 100 or len(cam) < 100:
    raise SystemExit(f'too few points: lidar {len(lidar)}, camera {len(cam)}')
cam = cam[::max(1, len(cam) // 1500)]
tree = cKDTree(lidar)
x = y = th = 0.0
for _ in range(80):
    c, s = math.cos(th), math.sin(th)
    moved = cam @ np.array([[c, s], [-s, c]]) + [x, y]
    d, idx = tree.query(moved, distance_upper_bound=0.25)
    ok = np.isfinite(d)
    if ok.sum() < 50:
        break
    keep = ok & (d < max(0.02, np.percentile(d[ok], 70)))
    p, q = moved[keep], lidar[idx[keep]]
    pm, qm = p.mean(axis=0), q.mean(axis=0)
    h = (p - pm).T @ (q - qm)
    dth = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    rot = np.array([[math.cos(dth), -math.sin(dth)], [math.sin(dth), math.cos(dth)]])
    dt = qm - rot @ pm
    th += dth
    x, y = rot @ [x, y] + dt
    if abs(dth) < 1e-6 and np.hypot(*dt) < 1e-6:
        break
c, s = math.cos(th), math.sin(th)
moved = cam @ np.array([[c, s], [-s, c]]) + [x, y]
d0, _ = tree.query(cam, distance_upper_bound=0.25)
d1, _ = tree.query(moved, distance_upper_bound=0.25)
print(f'nöqtələr: lidar {len(lidar)}, kamera {len(cam)}')
print(f'uyğunluq: düzəlişdən əvvəl median {np.median(d0[np.isfinite(d0)]) * 100:.1f} sm '
      f'({np.isfinite(d0).mean() * 100:.0f}% yaxın), sonra {np.median(d1[np.isfinite(d1)]) * 100:.1f} sm '
      f'({np.isfinite(d1).mean() * 100:.0f}% yaxın)')
reliable = (len(cam) >= 300 and abs(x) < 0.3 and abs(y) < 0.3 and abs(math.degrees(th)) < 10
            and np.median(d1[np.isfinite(d1)]) < 0.05)
if reliable:
    print(f'NƏTİCƏ: kameranın düzəlişi x {x * 100:+.1f} sm, y {y * 100:+.1f} sm, bucaq {math.degrees(th):+.1f}° '
          f'(bunları profildəki kamera mövqeyinə əlavə etmək lazımdır)')
else:
    print('NƏTİCƏ: ETİBARSIZ — kamera kifayət qədər divar/əşya görmür (≤4 m-də düz səthlər lazımdır); '
          'robotu divara 1-3 m məsafədə, ona baxacaq şəkildə qoyub təkrarlayın')
node.destroy_node()
rclpy.shutdown()
