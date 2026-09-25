import collections
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
# Must match RX_SYNC0/RX_SYNC1 in the ESP32 firmware. 0x34 marks the
# ticks/s command payload; firmware still expecting raw PWM (0x33) ignores it.
RX_SYNC = bytes([0xCC, 0x34])

ACCEL_SCALE = 1000.0        # raw / 1000.0 -> g
GYRO_SCALE = 572.957795     # raw / this -> rad/s  (deg/s*10 encoding)
ANGLE_SCALE = 5729.57795    # raw / this -> rad    (deg*100 encoding)
G_TO_MPS2 = 9.80665
INT16_MAX = 32767


def xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


def slew(current, target, accel, decel, dt):
    """Step current toward target: accel limits speeding up, decel everything else."""
    speeding_up = current * target >= 0.0 and abs(target) > abs(current)
    step = (accel if speeding_up else decel) * dt
    return current + max(-step, min(step, target - current))


class MotorBridgeNode(Node):
    """
    ROS2 <-> ESP32 bridge.

    Commands go out as per-wheel target speeds in encoder ticks/s, forward-positive
    on both wheels; the firmware owns motor mirroring and the speed loop.
    """

    def __init__(self):
        super().__init__('motor_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter_m', 0.09022)
        self.declare_parameter('wheel_separation_m', 0.44)
        self.declare_parameter('ticks_per_rev_left', 74)
        self.declare_parameter('ticks_per_rev_right', 73)
        self.declare_parameter('max_wheel_speed_mps', 0.3)
        self.declare_parameter('max_linear_accel_mps2', 0.3)
        self.declare_parameter('max_linear_decel_mps2', 1.0)
        self.declare_parameter('max_angular_accel_rps2', 1.0)
        self.declare_parameter('max_angular_decel_rps2', 2.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.3)
        self.declare_parameter('cmd_send_rate_hz', 20.0)
        self.declare_parameter('odom_publish_rate_hz', 50.0)
        self.declare_parameter('velocity_window_s', 0.1)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        self.port_name = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheel_separation = self.get_parameter('wheel_separation_m').value
        ticks_left = self.get_parameter('ticks_per_rev_left').value
        ticks_right = self.get_parameter('ticks_per_rev_right').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed_mps').value
        self.lin_accel = self.get_parameter('max_linear_accel_mps2').value
        self.lin_decel = self.get_parameter('max_linear_decel_mps2').value
        self.ang_accel = self.get_parameter('max_angular_accel_rps2').value
        self.ang_decel = self.get_parameter('max_angular_decel_rps2').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_s').value
        cmd_rate = self.get_parameter('cmd_send_rate_hz').value
        odom_rate = self.get_parameter('odom_publish_rate_hz').value
        self.velocity_window_ns = int(self.get_parameter('velocity_window_s').value * 1e9)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.meters_per_tick_left = (math.pi * wheel_diameter) / ticks_left
        self.meters_per_tick_right = (math.pi * wheel_diameter) / ticks_right
        self.cmd_period = 1.0 / cmd_rate
        # A step larger than half a meter between two ~20 ms samples can't be
        # real motion: the firmware rebooted and its counters restarted at 0.
        self.max_tick_jump = int(0.5 / self.meters_per_tick_left)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        # Only create the broadcaster when this node owns odom->base TF: the
        # constructor alone registers a (silent) /tf publisher.
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Shared between the serial reader thread and the timer callbacks.
        self._state_lock = threading.Lock()
        self._latest = None
        self._new_telemetry = False
        self._rebase = True

        # Timer-callback-only odometry state.
        self._prev_enc = None
        self._history = collections.deque()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        self._cmd_lock = threading.Lock()
        self._target_v = 0.0
        self._target_w = 0.0
        self._last_cmd_time = time.monotonic()
        self._v = 0.0
        self._w = 0.0

        self._serial_lock = threading.Lock()
        self._serial = None
        self._open_error_logged = False
        self._open_serial()

        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(self.cmd_period, self.send_cmd_callback)
        self.create_timer(1.0 / odom_rate, self.publish_odom_callback)

    # ---------------- serial connection ----------------

    def _open_serial(self):
        try:
            ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
        except (serial.SerialException, OSError) as exc:
            if not self._open_error_logged:
                self.get_logger().error(
                    f'Could not open {self.port_name}: {exc}; retrying every second')
                self._open_error_logged = True
            return False
        with self._serial_lock:
            self._serial = ser
        with self._state_lock:
            self._rebase = True
        self._open_error_logged = False
        self.get_logger().info(f'motor_bridge connected on {self.port_name} @ {self.baud} baud')
        return True

    def _close_serial(self):
        with self._serial_lock:
            ser, self._serial = self._serial, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass

    def _read_loop(self):
        buf = bytearray()
        while not self._stop_reader.is_set():
            ser = self._serial
            if ser is None:
                if not self._open_serial():
                    self._stop_reader.wait(1.0)
                    continue
                buf.clear()
                ser = self._serial
            try:
                # Read what has arrived instead of a fixed 256 bytes: waiting
                # for a full 256 delivered frames in ~44 ms batches that all
                # got the same timestamp.
                chunk = ser.read(max(1, ser.in_waiting))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial link lost ({exc}); reconnecting')
                self._close_serial()
                continue
            if not chunk:
                continue
            stamp = self.get_clock().now()
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
                if xor_checksum(frame[2:28]) == frame[28]:
                    self._handle_frame(frame, stamp)
                    del buf[:FRAME_LEN]
                else:
                    del buf[:2]

    def _handle_frame(self, frame: bytes, stamp):
        ax, ay, az, gx, gy, gz, roll, pitch, yaw = struct.unpack_from('<9h', frame, 2)
        enc_l, enc_r = struct.unpack_from('<ii', frame, 20)

        imu_fields = {
            'ax': ax / ACCEL_SCALE * G_TO_MPS2,
            'ay': ay / ACCEL_SCALE * G_TO_MPS2,
            'az': az / ACCEL_SCALE * G_TO_MPS2,
            'gx': gx / GYRO_SCALE,
            'gy': gy / GYRO_SCALE,
            'gz': gz / GYRO_SCALE,
            'yaw': yaw / ANGLE_SCALE,
        }

        with self._state_lock:
            self._latest = (stamp, enc_l, enc_r, imu_fields)
            self._new_telemetry = True

    # ---------------- cmd_vel -> serial ----------------

    def cmd_vel_callback(self, msg: Twist):
        with self._cmd_lock:
            self._target_v = msg.linear.x
            self._target_w = msg.angular.z
            self._last_cmd_time = time.monotonic()

    def send_cmd_callback(self):
        with self._cmd_lock:
            v_target = self._target_v
            w_target = self._target_w
            age = time.monotonic() - self._last_cmd_time
        if age > self.cmd_vel_timeout:
            v_target = 0.0
            w_target = 0.0

        # teleop publishes raw steps straight to cmd_vel (no velocity_smoother),
        # so acceleration is limited here too, before it can spin a wheel.
        self._v = slew(self._v, v_target, self.lin_accel, self.lin_decel, self.cmd_period)
        self._w = slew(self._w, w_target, self.ang_accel, self.ang_decel, self.cmd_period)

        v_left = self._v - self._w * self.wheel_separation / 2.0
        v_right = self._v + self._w * self.wheel_separation / 2.0
        peak = max(abs(v_left), abs(v_right))
        if peak > self.max_wheel_speed:
            # Scale both wheels together so the turn radius is preserved.
            scale = self.max_wheel_speed / peak
            v_left *= scale
            v_right *= scale

        self._send_targets(v_left / self.meters_per_tick_left,
                           v_right / self.meters_per_tick_right)

    def _send_targets(self, ticks_per_s_left: float, ticks_per_s_right: float):
        tl = max(-INT16_MAX, min(INT16_MAX, int(round(ticks_per_s_left))))
        tr = max(-INT16_MAX, min(INT16_MAX, int(round(ticks_per_s_right))))
        payload = struct.pack('<hh', tl, tr)
        frame = RX_SYNC + payload + bytes([xor_checksum(payload)])
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(frame)
            except (serial.SerialException, OSError):
                pass  # the reader thread notices the dead link and reconnects

    # ---------------- odometry / imu publishing ----------------

    def _rebase_to(self, stamp_ns, enc_l, enc_r):
        self._prev_enc = (enc_l, enc_r)
        self._history.clear()
        self._history.append((stamp_ns, enc_l, enc_r))

    def publish_odom_callback(self):
        with self._state_lock:
            if self._latest is None or not self._new_telemetry:
                return
            stamp, enc_l, enc_r, imu_fields = self._latest
            self._new_telemetry = False
            rebase = self._rebase
            self._rebase = False

        stamp_ns = stamp.nanoseconds
        self._publish_imu(stamp, imu_fields)

        if rebase or self._prev_enc is None:
            self._rebase_to(stamp_ns, enc_l, enc_r)
            return

        delta_l = enc_l - self._prev_enc[0]
        delta_r = enc_r - self._prev_enc[1]
        if abs(delta_l) > self.max_tick_jump or abs(delta_r) > self.max_tick_jump:
            self.get_logger().warning(
                f'Encoder jump of ({delta_l}, {delta_r}) ticks, assuming the ESP32 '
                'restarted; re-basing odometry instead of integrating it')
            self._rebase_to(stamp_ns, enc_l, enc_r)
            return
        self._prev_enc = (enc_l, enc_r)

        dist_l = delta_l * self.meters_per_tick_left
        dist_r = delta_r * self.meters_per_tick_right
        dist_center = (dist_l + dist_r) / 2.0
        dtheta = (dist_r - dist_l) / self.wheel_separation
        self._x += dist_center * math.cos(self._theta + dtheta / 2.0)
        self._y += dist_center * math.sin(self._theta + dtheta / 2.0)
        self._theta += dtheta

        # Velocity over a ~100 ms window: at walking speed a single 20 ms step
        # holds only 0-2 encoder ticks, far too coarse to fuse as a speed.
        self._history.append((stamp_ns, enc_l, enc_r))
        while len(self._history) > 2 and \
                stamp_ns - self._history[1][0] >= self.velocity_window_ns:
            self._history.popleft()
        t0, l0, r0 = self._history[0]
        span = (stamp_ns - t0) * 1e-9
        v = 0.0
        w = 0.0
        if span > 0.0:
            vl = (enc_l - l0) * self.meters_per_tick_left / span
            vr = (enc_r - r0) * self.meters_per_tick_right / span
            v = (vl + vr) / 2.0
            w = (vr - vl) / self.wheel_separation

        quat = Quaternion(x=0.0, y=0.0,
                          z=math.sin(self._theta / 2.0), w=math.cos(self._theta / 2.0))

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Wheel yaw is untrusted (slip); the EKF takes yaw rate from the IMU.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.001   # diff drive: sideways speed is ~0
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_imu(self, stamp, imu_fields):
        imu_msg = Imu()
        imu_msg.header.stamp = stamp.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = imu_fields['ax']
        imu_msg.linear_acceleration.y = imu_fields['ay']
        imu_msg.linear_acceleration.z = imu_fields['az']
        imu_msg.angular_velocity.x = imu_fields['gx']
        imu_msg.angular_velocity.y = imu_fields['gy']
        imu_msg.angular_velocity.z = imu_fields['gz']
        imu_msg.orientation.z = math.sin(imu_fields['yaw'] / 2.0)
        imu_msg.orientation.w = math.cos(imu_fields['yaw'] / 2.0)
        # Gyro-integrated yaw drifts (no magnetometer): mark orientation unusable.
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
        self._send_targets(0.0, 0.0)
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._close_serial()
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
        # rclpy's SIGINT handler may already have shut the context down.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
