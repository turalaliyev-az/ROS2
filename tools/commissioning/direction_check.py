"""
Which way is "forward"? Drive 0.3 m forward (through the collision monitor) and
compare: wheel encoders, the depth camera's centre distance, and the lidar's
0 deg and 180 deg distances. Run with the rover stack up. Usage: direction_check.py
"""
import math
import statistics
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

DIST, SPEED = 0.3, 0.15
rclpy.init()
node = rclpy.create_node('direction_check')
pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
st = {'odom': None, 'gz': [], 'scan': [], 'cam': [], 'collect': False}


def on_odom(msg):
    st['odom'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def on_imu(msg):
    st['gz'].append((time.monotonic(), msg.angular_velocity.z))


def sector(msg, centre, half_deg):
    vals = []
    for i, r in enumerate(msg.ranges):
        a = msg.angle_min + i * msg.angle_increment
        if abs(math.remainder(a - centre, 2 * math.pi)) <= math.radians(half_deg) \
                and r == r and msg.range_min < r < msg.range_max:
            vals.append(r)
    return vals


def on_scan(msg):
    if st['collect']:
        st['scan'].append((sector(msg, 0.0, 5), sector(msg, math.pi, 5)))


def on_cam(msg):
    if st['collect']:
        st['cam'].append(sector(msg, 0.0, 6))


node.create_subscription(Odometry, 'wheel_odom', on_odom, qos_profile_sensor_data)
node.create_subscription(Imu, 'imu/data_raw', on_imu, qos_profile_sensor_data)
node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
node.create_subscription(LaserScan, 'camera_scan', on_cam, qos_profile_sensor_data)


def spin_for(seconds, cmd=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cmd is not None:
            pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)


def median_of(lists):
    vals = [v for lst in lists for v in lst]
    return statistics.median(vals) if vals else float('nan')


def snapshot():
    st['scan'], st['cam'], st['collect'] = [], [], True
    spin_for(1.0, Twist())
    st['collect'] = False
    return (median_of([a for a, _ in st['scan']]), median_of([b for _, b in st['scan']]),
            median_of(st['cam']))


spin_for(1.0, Twist())
l0_a, l180_a, cam_a = snapshot()
x0, y0 = st['odom']
t0 = time.monotonic()
cmd = Twist()
cmd.linear.x = SPEED
try:
    while time.monotonic() - t0 < DIST / SPEED * 3 + 2:
        spin_for(0.05, cmd)
        gyro = sum((b[0] - a[0]) * a[1] for a, b in zip(st['gz'], st['gz'][1:]) if a[0] >= t0)
        if abs(gyro) > math.radians(10):
            print('⚠ 10°-dən çox döndü, dayandırıldı')
            break
        if math.hypot(st['odom'][0] - x0, st['odom'][1] - y0) >= DIST - 0.01:
            break
finally:
    spin_for(1.5, Twist())
l0_b, l180_b, cam_b = snapshot()
enc = math.hypot(st['odom'][0] - x0, st['odom'][1] - y0)
print(f'enkoder:     {enc:+.3f} m (əmr: irəli)')
print(f'kamera:      mərkəz {cam_a:.3f} → {cam_b:.3f} m  (kameraya tərəf = azalma {cam_a - cam_b:+.3f})')
print(f'lidar 0°:    {l0_a:.3f} → {l0_b:.3f} m  (0°-yə tərəf = azalma {l0_a - l0_b:+.3f})')
print(f'lidar 180°:  {l180_a:.3f} → {l180_b:.3f} m  (180°-yə tərəf = azalma {l180_a - l180_b:+.3f})')
toward_cam, toward_l0, toward_l180 = cam_a - cam_b, l0_a - l0_b, l180_a - l180_b
print('NƏTİCƏ:')
if toward_cam == toward_cam:
    print('  robot ' + ('KAMERAYA TƏRƏF getdi (irəli düzgündür)' if toward_cam > 0.05
                        else 'KAMERANIN ƏKSİNƏ getdi (irəli tərsdir)' if toward_cam < -0.05
                        else 'kamera ilə müəyyən edilmədi'))
print('  lidar ' + ('ilə hərəkət UYĞUNDUR (0° = irəli)' if toward_l0 > 0.05 or toward_l180 < -0.05
                    else 'ilə hərəkət ZİDDİR (180° = irəli)' if toward_l180 > 0.05 or toward_l0 < -0.05
                    else 'ilə müəyyən edilmədi'))
node.destroy_node()
rclpy.shutdown()
