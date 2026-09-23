import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

FRAME_LEN = 29
TX_SYNC = b'\xaa\x55'
RX_SYNC0 = 0xCC
RX_SYNC1 = 0x33

ACCEL_SCALE = 1000.0        # raw / 1000.0 -> g
GYRO_SCALE = 572.957795     # raw / this -> rad/s  (deg/s*10 encoding)
ANGLE_SCALE = 5729.57795    # raw / this -> rad    (deg*100 encoding)
G_TO_MPS2 = 9.80665

# Right motor is mechanically mirrored on this chassis: identical PWM signs
# spin the two wheels in physically opposite directions (confirmed empirically
# on 2026-09-21 by commanding L=R=200 and observing opposite wheel rotation),
# so the command sign must be flipped for the right side. The encoders are
# NOT mirrored relative to each other: for that same test enc_l=+1036 while
# enc_r=-572, i.e. enc_r went negative precisely when the right wheel spun
# backward -- so a positive tick delta already means "forward" on both
# sides once the command-side fix above is applied. Do not also flip the
# encoder sign here, or odometry will read a genuine straight drive as a
# turn.
LEFT_CMD_SIGN = 1
RIGHT_CMD_SIGN = -1


def xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


class MotorBridgeNode(Node):

    def __init__(self):
        super().__init__('motor_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter_m', 0.09022)
        self.declare_parameter('wheel_separation_m', 0.44)
        self.declare_parameter('ticks_per_rev_left', 74)
        self.declare_parameter('ticks_per_rev_right', 73)
        self.declare_parameter('max_pwm', 500)
        self.declare_parameter('max_wheel_speed_mps', 0.5)
        self.declare_parameter('cmd_vel_timeout_s', 0.3)
        self.declare_parameter('cmd_send_rate_hz', 20.0)
        self.declare_parameter('odom_publish_rate_hz', 50.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        self.port_name = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheel_separation = self.get_parameter('wheel_separation_m').value
        ticks_left = self.get_parameter('ticks_per_rev_left').value
        ticks_right = self.get_parameter('ticks_per_rev_right').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed_mps').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_s').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.meters_per_tick_left = (math.pi * wheel_diameter) / ticks_left
        self.meters_per_tick_right = (math.pi * wheel_diameter) / ticks_right

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        # Constructing TransformBroadcaster always registers a /tf
        # publisher endpoint, even if sendTransform() is never called --
        # only create it when this node is actually meant to own the TF
        # (publish_tf:=false when ekf_node is the TF source instead, e.g.
        # in bringup.launch.py), so the pub/sub graph doesn't show a
        # phantom, always-silent /tf publisher here.
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self._state_lock = threading.Lock()
        self._enc_l = None
        self._enc_r = None
        self._prev_enc_l = None
        self._prev_enc_r = None
        self._imu_fields = None
        self._new_telemetry = False

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = self.get_clock().now()

        self._cmd_lock = threading.Lock()
        self._target_v = 0.0
        self._target_w = 0.0
        self._last_cmd_time = time.monotonic()

        try:
            self.serial = serial.Serial(self.port_name, self.baud, timeout=0.05)
        except serial.SerialException as exc:
            self.get_logger().error(f'Could not open {self.port_name}: {exc}')
            raise

        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        cmd_rate = self.get_parameter('cmd_send_rate_hz').value
        odom_rate = self.get_parameter('odom_publish_rate_hz').value
        self.create_timer(1.0 / cmd_rate, self.send_cmd_callback)
        self.create_timer(1.0 / odom_rate, self.publish_odom_callback)

        self.get_logger().info(
            f'motor_bridge connected on {self.port_name} @ {self.baud} baud')

    # ---------------- serial reader thread ----------------

    def _read_loop(self):
        buf = bytearray()
        ser = self.serial
        while not self._stop_reader.is_set():
            try:
                chunk = ser.read(256)
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                idx = buf.find(TX_SYNC)
                if idx == -1:
                    if len(buf) > FRAME_LEN * 4:
                        del buf[:-2]
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < FRAME_LEN:
                    break
                frame = bytes(buf[:FRAME_LEN])
                payload = frame[2:28]
                checksum = frame[28]
                if xor_checksum(payload) == checksum:
                    self._handle_frame(frame)
                    del buf[:FRAME_LEN]
                else:
                    del buf[:2]

    def _handle_frame(self, frame: bytes):
        ax, ay, az, gx, gy, gz, roll, pitch, yaw = struct.unpack_from(
            '<9h', frame, 2)
        enc_l, enc_r = struct.unpack_from('<ii', frame, 20)

        imu_fields = {
            'ax': ax / ACCEL_SCALE * G_TO_MPS2,
            'ay': ay / ACCEL_SCALE * G_TO_MPS2,
            'az': az / ACCEL_SCALE * G_TO_MPS2,
            'gx': gx / GYRO_SCALE,
            'gy': gy / GYRO_SCALE,
            'gz': gz / GYRO_SCALE,
            'roll': roll / ANGLE_SCALE,
            'pitch': pitch / ANGLE_SCALE,
            'yaw': yaw / ANGLE_SCALE,
        }

        with self._state_lock:
            self._enc_l = enc_l
            self._enc_r = enc_r
            self._imu_fields = imu_fields
            self._new_telemetry = True

    # ---------------- cmd_vel -> serial ----------------

    def cmd_vel_callback(self, msg: Twist):
        with self._cmd_lock:
            self._target_v = msg.linear.x
            self._target_w = msg.angular.z
            self._last_cmd_time = time.monotonic()

    def send_cmd_callback(self):
        with self._cmd_lock:
            v = self._target_v
            w = self._target_w
            age = time.monotonic() - self._last_cmd_time
        if age > self.cmd_vel_timeout:
            v = 0.0
            w = 0.0

        v_left = v - w * self.wheel_separation / 2.0
        v_right = v + w * self.wheel_separation / 2.0

        pwm_left = int(max(-1.0, min(1.0, v_left / self.max_wheel_speed)) * self.max_pwm)
        pwm_right = int(max(-1.0, min(1.0, v_right / self.max_wheel_speed)) * self.max_pwm)

        self._send_pwm(pwm_left, pwm_right)

    def _send_pwm(self, pwm_left: int, pwm_right: int):
        raw_left = LEFT_CMD_SIGN * pwm_left
        raw_right = RIGHT_CMD_SIGN * pwm_right
        payload = struct.pack('<hh', raw_left, raw_right)
        chk = xor_checksum(payload)
        frame = bytes([RX_SYNC0, RX_SYNC1]) + payload + bytes([chk])
        try:
            self.serial.write(frame)
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    # ---------------- odometry / imu publishing ----------------

    def publish_odom_callback(self):
        with self._state_lock:
            if not self._new_telemetry or self._enc_l is None:
                return
            enc_l = self._enc_l
            enc_r = self._enc_r
            imu_fields = self._imu_fields
            self._new_telemetry = False

        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        if dt <= 0.0:
            return

        if self._prev_enc_l is None:
            self._prev_enc_l = enc_l
            self._prev_enc_r = enc_r
            return

        delta_l = enc_l - self._prev_enc_l
        delta_r = enc_r - self._prev_enc_r
        self._prev_enc_l = enc_l
        self._prev_enc_r = enc_r

        dist_l = delta_l * self.meters_per_tick_left
        dist_r = delta_r * self.meters_per_tick_right

        dist_center = (dist_l + dist_r) / 2.0
        dtheta = (dist_r - dist_l) / self.wheel_separation

        self._x += dist_center * math.cos(self._theta + dtheta / 2.0)
        self._y += dist_center * math.sin(self._theta + dtheta / 2.0)
        self._theta += dtheta

        v = dist_center / dt
        w = dtheta / dt

        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        quat = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Wheel-only estimate: position/vx are the only real signal here.
        # Yaw is left with high uncertainty since hand-spun or slipping
        # wheels report rotation that didn't actually happen -- the EKF
        # (see rover_bringup/config/ekf.yaml) is expected to trust the
        # IMU gyro for yaw rate instead and only take x/y/vx from here.
        odom.pose.covariance[0] = 0.01     # x
        odom.pose.covariance[7] = 0.01     # y
        odom.pose.covariance[35] = 0.2     # yaw
        odom.twist.covariance[0] = 0.01    # vx
        odom.twist.covariance[35] = 0.1    # vyaw
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf_msg)

        if imu_fields is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = imu_fields['ax']
            imu_msg.linear_acceleration.y = imu_fields['ay']
            imu_msg.linear_acceleration.z = imu_fields['az']
            imu_msg.angular_velocity.x = imu_fields['gx']
            imu_msg.angular_velocity.y = imu_fields['gy']
            imu_msg.angular_velocity.z = imu_fields['gz']
            iqz = math.sin(imu_fields['yaw'] / 2.0)
            iqw = math.cos(imu_fields['yaw'] / 2.0)
            imu_msg.orientation.z = iqz
            imu_msg.orientation.w = iqw
            # -1 tells consumers (robot_localization) that orientation is
            # not usable (yaw here is un-magnetometer-corrected gyro
            # integration, drifts over time) -- fuse angular_velocity.z
            # instead, which is set below with a real covariance.
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.0004
            imu_msg.linear_acceleration_covariance[0] = 0.04
            imu_msg.linear_acceleration_covariance[4] = 0.04
            imu_msg.linear_acceleration_covariance[8] = 0.04
            self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        self._stop_reader.set()
        self._send_pwm(0, 0)
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        try:
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler can already shut the context down
        # before this runs (observed under `ros2 launch` Ctrl-C: plain
        # rclpy.shutdown() then raised "rcl_shutdown already called").
        # try_shutdown() is the shutdown-if-not-already-shutdown idiom.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
