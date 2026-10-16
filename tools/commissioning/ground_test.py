"""
Ground test for the two-wheel platform: rotate in place or drive straight, and
measure the motion three independent ways (wheel encoders, ESP32 gyro, lidar).

Commands go through the collision monitor (cmd_vel_smoothed), so the lidar
still stops the robot before obstacles. Built-in aborts: translating while told
to rotate, the gyro turning the opposite way (left and right swapped), heading
change while told to drive straight. Run with the rover stack up.

Usage: ground_test.py rotate <deg, + = left> <rad/s>
       ground_test.py straight <m, + = forward> <m/s> [max heading change deg]
"""
import math
import statistics
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

mode, amount, rate = sys.argv[1], float(sys.argv[2]), abs(float(sys.argv[3]))
max_heading = math.radians(float(sys.argv[4])) if len(sys.argv) > 4 else math.radians(20)
rclpy.init()
node = rclpy.create_node('ground_test')
pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
st = {'yaw': None, 'enc_yaw': 0.0, 'x': 0.0, 'y': 0.0, 'gz': [], 'scans': [], 'collect': False,
      'wheels': []}
client = node.create_client(GetParameters, '/motor_bridge_node/get_parameters')
client.wait_for_service(timeout_sec=5.0)
fut = client.call_async(GetParameters.Request(names=['wheel_separation_m', 'reverse_drive']))
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
SEP = fut.result().values[0].double_value
# Wheels are reported in the robot's frame: with a reversed drive, the robot's
# left wheel is the drive's right one; the odometry is already in robot frame.
REVERSED = fut.result().values[1].bool_value


def on_odom(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if st['yaw'] is not None:
        st['enc_yaw'] += math.atan2(math.sin(yaw - st['yaw']), math.cos(yaw - st['yaw']))
    st['yaw'] = yaw
    st['x'], st['y'] = msg.pose.pose.position.x, msg.pose.pose.position.y
    v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
    st['wheels'].append((time.monotonic(), v - w * SEP / 2, v + w * SEP / 2))


def on_imu(msg):
    st['gz'].append((time.monotonic(), msg.angular_velocity.z))


def on_scan(msg):
    if st['collect']:
        st['scans'].append(list(msg.ranges))
    st['scan_meta'] = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)


node.create_subscription(Odometry, 'wheel_odom', on_odom, qos_profile_sensor_data)
node.create_subscription(Imu, 'imu/data_raw', on_imu, qos_profile_sensor_data)
node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)


def spin_for(seconds, cmd=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cmd is not None:
            pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)


def rest_scan(n=6):
    """Median of n scans taken at rest, beam by beam (NaN where unreliable)."""
    st['scans'], st['collect'] = [], True
    while len(st['scans']) < n:
        spin_for(0.05, Twist())
    st['collect'] = False
    _, _, rmin, rmax = st['scan_meta']
    out = []
    for beam in zip(*st['scans']):
        good = [r for r in beam if r == r and rmin < r < rmax]
        out.append(statistics.median(good) if len(good) >= n // 2 + 1 else float('nan'))
    return out


def scan_rotation(before, after, guess_rad):
    """Rotation (rad, + = left) that best maps `after` onto `before` (circular shift)."""
    _, inc, _, _ = st['scan_meta']
    n = len(before)

    def cost(shift):
        diffs = [abs(after[i] - before[(i + shift) % n]) for i in range(n)
                 if after[i] == after[i] and before[(i + shift) % n] == before[(i + shift) % n]]
        return statistics.median(diffs) if len(diffs) > n // 4 else float('inf')

    # Whole circle: no assumption about which way it turned.
    costs = {s: cost(s) for s in range(n)}
    best = min(costs, key=costs.get)
    a, b, c = costs[(best - 1) % n], costs[best], costs[(best + 1) % n]
    frac = 0.0
    if math.isfinite(a) and math.isfinite(c) and (a - 2 * b + c) > 0:
        frac = 0.5 * (a - c) / (a - 2 * b + c)
    theta = math.remainder((best + frac) * inc, 2 * math.pi)
    # Whole turns are invisible to a scan: take them from the gyro.
    theta += 2 * math.pi * round((guess_rad - theta) / (2 * math.pi))
    return theta, costs[best]


def sector(scan, centre_rad, half_deg=4):
    a0, inc, _, _ = st['scan_meta']
    n = len(scan)
    vals = [scan[round((centre_rad + math.radians(d) - a0) / inc) % n]
            for d in range(-half_deg, half_deg + 1)]
    vals = [v for v in vals if v == v]
    return statistics.median(vals) if len(vals) >= 3 else float('nan')


def gyro_bias():
    samples = [g for t, g in st['gz'][-60:]]
    return statistics.mean(samples) if samples else 0.0


def gyro_turn(since, bias):
    pts = [(t, g) for t, g in st['gz'] if t >= since]
    return sum((b[0] - a[0]) * (a[1] - bias) for a, b in zip(pts, pts[1:]))


stop = Twist()
aborted = None
cmd = Twist()
try:
    spin_for(1.5, stop)  # settle; gyro bias from the rest period
    if st['yaw'] is None or 'scan_meta' not in st:
        sys.exit('no wheel_odom or scan data')
    bias = gyro_bias()
    before = rest_scan()
    x0, y0, yaw0 = st['x'], st['y'], st['enc_yaw']
    t0 = time.monotonic()
    if mode == 'rotate':
        target = math.radians(amount)
        cmd.angular.z = math.copysign(rate, target)
        limit = abs(target) / rate * 2.5 + 2.0
        print(f'>>> yerində dönmə {amount:+.0f}° ({"sola" if amount > 0 else "sağa"}), {rate} rad/s',
              flush=True)
        while time.monotonic() - t0 < limit:
            spin_for(0.05, cmd)
            turned = st['enc_yaw'] - yaw0
            moved = math.hypot(st['x'] - x0, st['y'] - y0)
            g = gyro_turn(t0, bias)
            if moved > 0.15:
                aborted = f'robot {moved:.2f} m yerini dəyişdi (təmiz dönmə deyil)'
                break
            if time.monotonic() - t0 > 1.5 and abs(g) > 0.15 and g * target < 0:
                aborted = 'giroskop ƏKS istiqamətə dönmə göstərir: sol/sağ mühərrik qarışıb'
                break
            if abs(turned) >= abs(target) - math.radians(2):
                break
    else:
        target = amount
        cmd.linear.x = math.copysign(rate, target)
        limit = abs(target) / rate * 2.5 + 2.0
        print(f'>>> düz {"irəli" if amount > 0 else "geri"} {abs(amount):.2f} m, {rate} m/s', flush=True)
        while time.monotonic() - t0 < limit:
            spin_for(0.05, cmd)
            dist = math.hypot(st['x'] - x0, st['y'] - y0)
            if abs(st['enc_yaw'] - yaw0) > max_heading or abs(gyro_turn(t0, bias)) > max_heading:
                aborted = (f'düz getmir ({math.degrees(max_heading):.0f}°-dən çox döndü)')
                break
            if dist >= abs(target) - 0.01:
                break
    elapsed = time.monotonic() - t0
    t_stop = time.monotonic()
finally:
    spin_for(1.5, stop)

after = rest_scan()
enc_turn = st['enc_yaw'] - yaw0
g_turn = gyro_turn(t0, bias)
moved = math.hypot(st['x'] - x0, st['y'] - y0)
print(f'    müddət {elapsed:.1f} s' + (f'  ⚠ DAYANDIRILDI: {aborted}' if aborted else ''))
print(f'    enkoder: dönmə {math.degrees(enc_turn):+7.1f}°, yerdəyişmə {moved:.3f} m')
print(f'    giroskop: dönmə {math.degrees(g_turn):+7.1f}°  (bias {math.degrees(bias):+.2f}°/s)')
cmd_l = cmd.linear.x - cmd.angular.z * SEP / 2
cmd_r = cmd.linear.x + cmd.angular.z * SEP / 2
for name, idx, target_v in (('SOL', 1, cmd_l), ('SAĞ', 2, cmd_r)):
    seg = [w for w in st['wheels'] if t0 <= w[0] <= t0 + elapsed]
    signed = sum(b[idx] * (b[0] - a[0]) for a, b in zip(seg, seg[1:]))
    dist = sum(abs(b[idx]) * (b[0] - a[0]) for a, b in zip(seg, seg[1:]))
    first = next((w[0] - t0 for w in seg if abs(w[idx]) > 0.01), None)
    still = sum(1 for w in seg if abs(w[idx]) < 0.01) / max(1, len(seg))
    mean = signed / elapsed if elapsed else 0.0
    verdict = ('FIRLANMADI' if dist < 0.02 else 'ƏKS istiqamətə' if signed * target_v < 0
               else 'gec başladı' if first and first > 1.0 else 'işlədi')
    print(f'    {name} təkər: hədəf {target_v:+.3f} m/s, orta {mean:+.3f} m/s, yol {signed:+.3f} m, '
          f'ilk hərəkət {"—" if first is None else f"{first:.2f} s"}, dayanıqlı {100 * still:.0f}% → {verdict}')
coast = [w for w in st['wheels'] if w[0] >= t_stop]
moving_after = next((w[0] - t_stop for w in reversed(coast) if abs(w[1]) > 0.01 or abs(w[2]) > 0.01), 0.0)
print(f'    dayanma: STOP əmrindən sonra {moving_after:.2f} s hərəkət etdi')
if mode == 'rotate':
    lid, quality = scan_rotation(before, after, g_turn)
    print(f'    lidar:   dönmə {math.degrees(lid):+7.1f}°  (uyğunluq xətası {quality:.3f} m)')
    if abs(lid) > math.radians(5):
        print(f'    → enkoder/lidar = {enc_turn / lid:.3f} (1.000 = iz düzgündür)')
else:
    f0, f1 = sector(before, 0.0), sector(after, 0.0)
    b0, b1 = sector(before, math.pi), sector(after, math.pi)
    parts = []
    if f0 == f0 and f1 == f1:
        parts.append(f0 - f1)
    if b0 == b0 and b1 == b1:
        parts.append(b1 - b0)
    print(f'    lidar: öndə {f0:.3f}→{f1:.3f} m, arxada {b0:.3f}→{b1:.3f} m')
    if abs(g_turn) > math.radians(3):
        print('    lidar yol: hesablanmadı (robot düz getmədi)')
    elif parts:
        lid = statistics.mean(parts)
        print(f'    lidar yol {lid:+.3f} m → enkoder/lidar = {moved * math.copysign(1, target) / lid:.3f} '
              f'(1.000 = diametr düzgündür; >1: diametr çox böyük verilib)')
print('>>> STOP', flush=True)
node.destroy_node()
rclpy.shutdown()
