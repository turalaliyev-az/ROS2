"""
Camera height and tilt from the floor, seen by the depth camera itself (robot
standing still, open floor in front of it; rover service running).

RANSAC finds the floor plane in the depth point cloud (optical frame: x right,
y down, z forward). Its distance from the camera is the camera's height above
the floor; its normal gives how far the camera looks down (pitch) and leans
sideways (roll).
Usage: camera_floor.py
"""
import math
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

rclpy.init()
node = rclpy.create_node('camera_floor')
clouds = []
node.create_subscription(PointCloud2, '/camera/camera/depth/color/points',
                         lambda m: clouds.append(m) if len(clouds) < 5 else None, qos_profile_sensor_data)
end = time.monotonic() + 15
while time.monotonic() < end and len(clouds) < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
if not clouds:
    raise SystemExit('no depth point cloud')
pts = []
for c in clouds:
    a = point_cloud2.read_points_numpy(c, field_names=('x', 'y', 'z'), skip_nans=True)
    pts.append(a[::7])
pts = np.concatenate(pts).astype(float)
frame = clouds[0].header.frame_id
# candidate floor points: below the optical axis, 0.3-3 m ahead
cand = pts[(pts[:, 1] > 0.02) & (pts[:, 2] > 0.3) & (pts[:, 2] < 3.0)]
if len(cand) < 500:
    raise SystemExit(f'too few points below the horizon ({len(cand)}): is the floor in view?')
rng = np.random.default_rng(0)
best = (0, None)
for _ in range(400):
    s = cand[rng.choice(len(cand), 3, replace=False)]
    n = np.cross(s[1] - s[0], s[2] - s[0])
    if np.linalg.norm(n) < 1e-6:
        continue
    n /= np.linalg.norm(n)
    if n[1] > 0:
        n = -n                       # point the normal up (optical y is down)
    if -n[1] < 0.8:
        continue                     # not floor-like (tilted > ~37 deg)
    d = -n @ s[0]
    inliers = np.abs(cand @ n + d) < 0.015
    if inliers.sum() > best[0]:
        best = (int(inliers.sum()), inliers)
inl = cand[best[1]]
# least-squares refit on the inliers
centroid = inl.mean(axis=0)
_, _, vt = np.linalg.svd(inl - centroid)
n = vt[2] / np.linalg.norm(vt[2])
if n[1] > 0:
    n = -n
d = -n @ centroid
height = abs(d)
pitch = math.degrees(math.atan2(-n[2], -n[1]))   # + = looking down
roll = math.degrees(math.atan2(n[0], -n[1]))
spread = float(np.std(inl @ n + d))
print(f'kadr: {frame}; nöqtələr {len(pts)}, döşəmə namizədi {len(cand)}, döşəmədə {len(inl)} '
      f'({100 * len(inl) / len(cand):.0f}%), müstəvi səpələnməsi {spread * 1000:.1f} mm')
print(f'NƏTİCƏ: kamera döşəmədən {height * 100:.1f} sm hündürdə; aşağı əyilmə {pitch:+.1f}°, '
      f'yan əyilmə {roll:+.1f}°')
node.destroy_node()
rclpy.shutdown()
