"""
Odometry calibration against the lidar (rover service running; commands go
through the collision monitor).

Ground truth comes from scan matching (2D ICP) between scans taken at rest
before and after each move, so it uses the whole room, not one sector.
Moves: turn +360 and -360 deg in place, drive +D and -D straight.
Results: wheel diameter (from the straight moves), effective track (from the
turns, with the corrected diameter) and the left/right diameter difference
(from the heading change on the straight moves).
Usage: calibrate_odometry.py [distance m] [--apply]
"""
import json
import warnings
import math
import pathlib
import statistics
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import Imu, LaserScan

DIST = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 1.0
TURN_RATE, DRIVE_SPEED = 0.4, 0.2
LOG = pathlib.Path(__file__).parent / 'logs'

rclpy.init()
node = rclpy.create_node('calibrate_odometry')
pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
st = {'yaw': None, 'enc_yaw': 0.0, 'x': 0.0, 'y': 0.0, 'gz': [], 'scans': [], 'collect': False}
cli = node.create_client(GetParameters, '/motor_bridge_node/get_parameters')
cli.wait_for_service(timeout_sec=5.0)
fut = cli.call_async(GetParameters.Request(names=['wheel_separation_m', 'wheel_diameter_m']))
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
SEP, DIAM = (v.double_value for v in fut.result().values)


def on_odom(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if st['yaw'] is not None:
        st['enc_yaw'] += math.atan2(math.sin(yaw - st['yaw']), math.cos(yaw - st['yaw']))
    st['yaw'] = yaw
    st['x'], st['y'] = msg.pose.pose.position.x, msg.pose.pose.position.y


def on_imu(msg):
    st['gz'].append((time.monotonic(), msg.angular_velocity.z))


def on_scan(msg):
    st['meta'] = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
    if st['collect']:
        st['scans'].append((msg.angle_min, msg.angle_increment, np.asarray(msg.ranges, dtype=float)))


node.create_subscription(Odometry, 'wheel_odom', on_odom, qos_profile_sensor_data)
node.create_subscription(Imu, 'imu/data_raw', on_imu, qos_profile_sensor_data)
node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)


def spin_for(seconds, cmd=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cmd is not None:
            pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)


def rest_points(n=8):
    """Median scan at rest as Nx2 points in the lidar frame."""
    spin_for(1.0, Twist())
    st['scans'], st['collect'] = [], True
    while len(st['scans']) < n:
        spin_for(0.05, Twist())
    st['collect'] = False
    # The LD19's beam count changes from scan to scan: put every scan on a
    # fixed 0.72 deg grid, then take the median per grid cell.
    _, _, rmin, rmax = st['meta']
    cells = 500
    grid = np.full((len(st['scans']), cells), np.nan)
    for k, (a0, inc, r) in enumerate(st['scans']):
        ang = (a0 + inc * np.arange(len(r))) % (2 * math.pi)
        idx = np.minimum((ang / (2 * math.pi) * cells).astype(int), cells - 1)
        good = np.isfinite(r) & (r > rmin) & (r < min(rmax, 12.0))
        grid[k, idx[good]] = r[good]
    with np.errstate(all='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        med = np.nanmedian(grid, axis=0)
    ang = (np.arange(cells) + 0.5) * 2 * math.pi / cells
    ok = np.isfinite(med) & (np.sum(np.isfinite(grid), axis=0) >= n // 2)
    return np.stack([med[ok] * np.cos(ang[ok]), med[ok] * np.sin(ang[ok])], axis=1)


def icp(src, dst, guess, iters=60):
    """Rigid (x, y, theta) moving the robot from `dst`'s pose to `src`'s: src ≈ R(-θ)(dst - t)."""
    # Points seen after the move (src) mapped into the frame before it.
    tree = cKDTree(dst)
    x, y, th = guess
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        moved = src @ np.array([[c, s], [-s, c]]) + [x, y]
        d, idx = tree.query(moved, distance_upper_bound=0.3)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            break
        keep = ok & (d < max(0.03, np.percentile(d[ok], 80)))
        p, q = moved[keep], dst[idx[keep]]
        pm, qm = p.mean(axis=0), q.mean(axis=0)
        h = (p - pm).T @ (q - qm)
        dth = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        cd, sd = math.cos(dth), math.sin(dth)
        rot = np.array([[cd, -sd], [sd, cd]])
        dt = qm - rot @ pm
        # compose the increment onto the current estimate
        th += dth
        x, y = rot @ [x, y] + dt
        if abs(dth) < 1e-5 and np.hypot(*dt) < 1e-5:
            break
    c, s = math.cos(th), math.sin(th)
    moved = src @ np.array([[c, s], [-s, c]]) + [x, y]
    d, _ = tree.query(moved, distance_upper_bound=0.3)
    fit = float(np.median(d[np.isfinite(d)])) if np.isfinite(d).any() else float('nan')
    return x, y, th, fit, int(np.isfinite(d).sum())


def gyro_turn(since):
    pts = [(t, g) for t, g in st['gz'] if t >= since]
    return sum((b[0] - a[0]) * a[1] for a, b in zip(pts, pts[1:]))


def move(kind, amount):
    before = rest_points()
    x0, y0, yaw0 = st['x'], st['y'], st['enc_yaw']
    t0 = time.monotonic()
    cmd = Twist()
    if kind == 'turn':
        cmd.angular.z = math.copysign(TURN_RATE, amount)
        limit = abs(amount) / TURN_RATE * 2.5 + 3
        while time.monotonic() - t0 < limit and abs(st['enc_yaw'] - yaw0) < abs(amount) - 0.03:
            spin_for(0.05, cmd)
    else:
        cmd.linear.x = math.copysign(DRIVE_SPEED, amount)
        limit = abs(amount) / DRIVE_SPEED * 2.5 + 3
        while time.monotonic() - t0 < limit and math.hypot(st['x'] - x0, st['y'] - y0) < abs(amount) - 0.02:
            spin_for(0.05, cmd)
    after = rest_points()
    enc_yaw = st['enc_yaw'] - yaw0
    gyro = gyro_turn(t0)
    # encoder translation in the frame of the start pose
    c, s = math.cos(-st['yaw'] + enc_yaw), math.sin(-st['yaw'] + enc_yaw)
    dx, dy = st['x'] - x0, st['y'] - y0
    yaw_start = st['yaw'] - enc_yaw
    ex = math.cos(yaw_start) * dx + math.sin(yaw_start) * dy
    ey = -math.sin(yaw_start) * dx + math.cos(yaw_start) * dy
    # ICP from the encoder translation and the gyro rotation (the gyro is within
    # ~2 % of the lidar; the encoders were ~10 % off, i.e. ~35 deg per turn)
    lx, ly, lth, fit, n = icp(after, before, (ex, ey, math.remainder(gyro, 2 * math.pi)))
    lth += 2 * math.pi * round((gyro - lth) / (2 * math.pi))
    res = {'kind': kind, 'amount': amount, 'enc': (round(ex, 4), round(ey, 4), round(enc_yaw, 4)),
           'lidar': (round(lx, 4), round(ly, 4), round(lth, 4)), 'gyro': round(gyro, 4),
           'fit_m': round(fit, 4), 'points': n}
    print(f'  {kind} {amount:+.2f}: enkoder ({ex:+.3f}, {ey:+.3f} m, {math.degrees(enc_yaw):+7.1f}°) | '
          f'lidar ({lx:+.3f}, {ly:+.3f} m, {math.degrees(lth):+7.1f}°) | giroskop {math.degrees(gyro):+7.1f}° | '
          f'ICP xətası {fit * 100:.1f} sm, {n} nöqtə', flush=True)
    return res


try:
    deadline = time.monotonic() + 10.0   # discovery of a fresh node can take a few seconds
    while (st['yaw'] is None or 'meta' not in st) and time.monotonic() < deadline:
        spin_for(0.2, Twist())
    if st['yaw'] is None or 'meta' not in st:
        sys.exit('no wheel_odom or scan')
    if '--offset' in sys.argv:
        print('>>> lidarın dönmə mərkəzinə görə yeri: 180° sola, 180° sağa', flush=True)
        results = [move('turn', math.pi), move('turn', -math.pi)]
    else:
        print(f'>>> parametrlər: diametr {DIAM} m, iz {SEP} m; hərəkətlər: 2 dönmə, 2 düz ({DIST} m)',
              flush=True)
        results = [move('turn', 2 * math.pi), move('turn', -2 * math.pi),
                   move('drive', DIST), move('drive', -DIST)]
finally:
    spin_for(1.0, Twist())

if '--offset' in sys.argv:
    # The lidar sits at o from the rotation centre: turning by theta moves it
    # by (R(theta) - I) o, seen in the start frame.
    offs = []
    for r in results:
        lx, ly, th = r['lidar']
        a, b, c, d = math.cos(th) - 1, -math.sin(th), math.sin(th), math.cos(th) - 1
        det = a * d - b * c
        ox, oy = (d * lx - b * ly) / det, (-c * lx + a * ly) / det
        offs.append((ox, oy))
        print(f'  dönmə {math.degrees(th):+.1f}°: lidar ofseti x {ox * 100:+.1f} sm, y {oy * 100:+.1f} sm '
              f'(ICP xətası {r["fit_m"] * 100:.1f} sm)')
    ox = statistics.mean(o[0] for o in offs)
    oy = statistics.mean(o[1] for o in offs)
    print(f'NƏTİCƏ: lidar dönmə mərkəzindən x {ox * 100:+.1f} sm (önə +), y {oy * 100:+.1f} sm (sola +)')
    LOG.mkdir(exist_ok=True)
    with open(LOG / 'lidar_offset.jsonl', 'a') as f:
        f.write(json.dumps({'time': time.strftime('%F %T'), 'results': results, 'offset': (ox, oy)}) + '\n')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def valid(r):
    return r['fit_m'] < 0.06 and r['points'] >= 100


for r in results:
    if not valid(r):
        print(f"  ⚠ {r['kind']} {r['amount']:+.2f} atıldı: ICP etibarsızdır "
              f"(xəta {r['fit_m'] * 100:.1f} sm, {r['points']} nöqtə) — maneə və ya dəyişən səhnə")
turns = [r for r in results if r['kind'] == 'turn' and valid(r)]
drives = [r for r in results if r['kind'] == 'drive' and valid(r)]
if not turns or not drives:
    sys.exit('kalibrləmə üçün kifayət qədər etibarlı ölçmə yoxdur')
enc_d = [math.hypot(*r['enc'][:2]) for r in drives]
lid_d = [math.hypot(*r['lidar'][:2]) for r in drives]
scale_d = statistics.mean(l / e for l, e in zip(lid_d, enc_d))
d_true = DIAM * scale_d
yaw_ratio = statistics.mean(r['enc'][2] / r['lidar'][2] for r in turns)
sep_true = SEP * scale_d * yaw_ratio
# heading change while "straight": right wheel larger than left -> turns left
eps = statistics.mean(r['lidar'][2] * sep_true / (math.hypot(*r['lidar'][:2]) * math.copysign(1, r['amount']))
                      for r in drives)
print(f'NƏTİCƏ:\n  diametr: {DIAM:.4f} → {d_true:.4f} m (lidar/enkoder yol {scale_d:.4f})')
print(f'  iz:      {SEP:.4f} → {sep_true:.4f} m (enkoder/lidar dönmə {yaw_ratio:.4f})')
print(f'  sağ/sol diametr fərqi: {eps * 100:+.2f} % (düz gedişdə lidar dönməsindən)')
print(f'  giroskop/lidar dönmə: {statistics.mean(r["gyro"] / r["lidar"][2] for r in turns):.4f}')
LOG.mkdir(exist_ok=True)
with open(LOG / 'odometry_calibration.jsonl', 'a') as f:
    f.write(json.dumps({'time': time.strftime('%F %T'), 'diam': DIAM, 'sep': SEP, 'results': results,
                        'd_true': d_true, 'sep_true': sep_true, 'lr_diff': eps}) + '\n')
node.destroy_node()
rclpy.shutdown()
