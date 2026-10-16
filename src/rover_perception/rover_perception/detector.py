"""
Object recognition: people, chairs and tables from the RealSense camera.

YOLO, pretrained on COCO (these classes need no training), runs on the colour
image; the depth image gives each object a 3D position; a small tracker keeps
ids stable and positions smooth in the odom frame.

Publishes
  /perception/objects       std_msgs/String, JSON for the web panel. Per
                            object: 'dist' (m to its near side) and, when
                            known, 'base' / 'map': the near side's (x, y) in
                            base_footprint / map.
  /perception/people_cloud  sensor_msgs/PointCloud2 in odom: each person seen
                            within person_max_age as a ring of points, for
                            Nav2's collision monitor (slow down near people,
                            wait for them to pass). Sent at a fixed rate even
                            when empty or when the camera or model is missing:
                            the collision monitor stops the robot when one of
                            its sources goes silent.
  /perception/status        std_msgs/String (latched)
Listens to
  /perception/destination   geometry_msgs/PoseStamped (latched), from the web
                            server: where the robot is heading. People close
                            to it (customers at the destination table) are
                            left out of the cloud, or the robot could never
                            arrive. An empty frame_id means no destination.
  /perception/people_guard  std_msgs/Bool (latched); false = always empty cloud
"""
import json
import os
import threading
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rover_perception.geometry import (depth_points, locate, person_rings, project,
                                       quat_to_matrix, scale_intrinsics, transform)
from rover_perception.tracker import Tracker
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

# COCO class ids the robot cares about.
CLASSES = {0: 'person', 56: 'chair', 57: 'sofa', 60: 'table'}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
RING_POINTS = 12
RING_HEIGHT = 0.5  # m; inside the collision monitor's people source height range


def image_to_bgr(msg):
    """Turn a colour Image message into an HxWx3 BGR array."""
    channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
    data = np.frombuffer(msg.data, dtype=np.uint8)
    img = data.reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels)[:, :, :3]
    if msg.encoding.startswith('rgb'):
        img = img[:, :, ::-1]  # the model takes OpenCV-style BGR
    return np.ascontiguousarray(img)


def depth_to_metres(msg):
    """Turn a depth Image message (16UC1 mm or 32FC1 m) into an HxW float32 array in metres."""
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(msg.data, dtype='>u2' if msg.is_bigendian else '<u2')
        return raw.reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        raw = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4')
        depth = raw.reshape(msg.height, msg.step // 4)[:, :msg.width].astype(np.float32)
        return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    raise ValueError(f'unsupported depth encoding {msg.encoding}')


def make_cloud(points, frame, stamp):
    """Build an x, y, z float32 PointCloud2 from Nx3 points."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class Detector(Node):
    """Runs YOLO on the camera and publishes objects, people and status."""

    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('model_path', '~/.rover/models/yolo11n.pt')
        self.declare_parameter('rate_hz', 8.0)
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('person_radius', 0.3)
        self.declare_parameter('person_max_age', 1.0)
        self.declare_parameter('goal_exclusion_radius', 1.2)
        # A destination nobody renewed for this long is ignored (e.g. the web
        # server died mid-trip): better a robot that waits than one that
        # ignores people.
        self.declare_parameter('destination_max_age', 300.0)
        self.declare_parameter('cloud_rate_hz', 10.0)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')

        p = self.get_parameter
        self.model_path = os.path.expanduser(p('model_path').value)
        self.rate = p('rate_hz').value
        self.conf = p('confidence').value
        self.person_radius = p('person_radius').value
        self.person_max_age = p('person_max_age').value
        self.goal_exclusion = p('goal_exclusion_radius').value
        self.destination_max_age = p('destination_max_age').value
        color_topic = p('color_topic').value
        depth_topic = p('depth_topic').value

        self.lock = threading.Lock()
        self.color = None
        self.color_seq = 0
        self.color_time = 0.0
        self.depth = None
        self.depth_time = 0.0
        self.info = {}           # 'color' | 'depth' -> (K, width, height) from camera_info
        self.people = []         # [(x, y, seen)] in odom; seen: monotonic time
        self.destination = None  # (PoseStamped, monotonic time received)
        self.guard = True
        self.status = ''

        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, color_topic.rsplit('/', 1)[0] + '/camera_info',
                                 self._info_callback('color'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, depth_topic.rsplit('/', 1)[0] + '/camera_info',
                                 self._info_callback('depth'), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'perception/destination', self._on_destination,
                                 LATCHED)
        self.create_subscription(Bool, 'perception/people_guard', self._on_guard, LATCHED)

        self.objects_pub = self.create_publisher(String, 'perception/objects', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'perception/people_cloud', 10)
        self.status_pub = self.create_publisher(String, 'perception/status', LATCHED)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracker = Tracker(match_radius=0.6, max_age=self.person_max_age)

        self.create_timer(1.0 / p('cloud_rate_hz').value, self._publish_cloud)
        self._set_status('loading')
        threading.Thread(target=self._run, daemon=True).start()

    # ---------------- inputs ----------------

    def _on_color(self, msg):
        with self.lock:
            self.color = msg
            self.color_seq += 1
            self.color_time = time.monotonic()

    def _on_depth(self, msg):
        with self.lock:
            self.depth = msg
            self.depth_time = time.monotonic()

    def _info_callback(self, kind):
        def callback(msg):
            self.info[kind] = (list(msg.k), msg.width, msg.height)
        return callback

    def _on_destination(self, msg):
        self.destination = (msg, time.monotonic()) if msg.header.frame_id else None

    def _on_guard(self, msg):
        self.guard = msg.data

    def _set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(f'status: {text}')

    def _intrinsics(self, kind, width, height):
        """K for an image of this size, or None before camera_info arrives."""
        info = self.info.get(kind)
        if info is None:
            return None
        k, info_width, info_height = info
        return scale_intrinsics(k, (info_width, info_height), (width, height))

    def _tf(self, target, source, stamp=None):
        """
        Get (rotation, translation) taking points from `source` into `target`, or None.

        At `stamp` (the image's time) when given and available, so motion
        between taking the picture and processing it doesn't shift positions;
        otherwise the latest transform.
        """
        times = [rclpy.time.Time()]
        if stamp is not None:
            times.insert(0, rclpy.time.Time.from_msg(stamp))
        for when in times:
            try:
                t = self.tf_buffer.lookup_transform(target, source, when)
            except TransformException:
                continue
            q, tr = t.transform.rotation, t.transform.translation
            return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])
        return None

    # ---------------- recognition ----------------

    def _load_model(self):
        try:
            import torch
            from ultralytics import YOLO
        except Exception as exc:  # noqa: B902 - any import failure means no model
            self._set_status(f'error: ultralytics/torch not available ({exc})')
            return None, None
        if not os.path.exists(self.model_path):
            self._set_status(f'error: model file missing ({self.model_path})')
            return None, None
        device = 0 if torch.cuda.is_available() else 'cpu'
        try:
            model = YOLO(self.model_path)
            model.predict(np.zeros((480, 640, 3), dtype=np.uint8), device=device, verbose=False)
        except Exception as exc:  # noqa: B902
            self._set_status(f'error: model failed to load ({exc})')
            return None, None
        if device == 'cpu':
            self.rate = min(self.rate, 3.0)
        return model, device

    def _run(self):
        model, device = self._load_model()
        if model is None:
            return  # status says why; the empty people cloud keeps going
        self._set_status('ok')
        last_seq = -1
        while rclpy.ok():
            started = time.monotonic()
            with self.lock:
                color, seq, color_time = self.color, self.color_seq, self.color_time
                depth = self.depth if started - self.depth_time < 0.5 else None
            if color is None or started - color_time > 2.0:
                # People not seen again drop out of the cloud by themselves
                # (person_max_age): no stale person keeps the robot waiting.
                self._set_status('no camera')
                time.sleep(0.2)
                continue
            if seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq
            try:
                self._process(model, device, color, depth)
                self._set_status('ok' if device == 0 else 'ok (CPU)')
            except Exception as exc:  # noqa: B902 - keep running, report it
                self._set_status(f'error: {exc}')
                self.get_logger().warning(f'recognition failed: {exc}')
            time.sleep(max(0.0, 1.0 / self.rate - (time.monotonic() - started)))

    def _process(self, model, device, color, depth):
        img = image_to_bgr(color)
        h, w = img.shape[:2]
        result = model.predict(img, device=device, conf=self.conf, classes=list(CLASSES),
                               imgsz=640, verbose=False)[0]
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        scores = result.boxes.conf.cpu().numpy()

        stamp = color.header.stamp
        color_frame = color.header.frame_id
        k_color = self._intrinsics('color', w, h)
        pts = pixels = None
        if depth is not None and k_color is not None:
            k_depth = self._intrinsics('depth', depth.width, depth.height)
            to_color = self._tf(color_frame, depth.header.frame_id) if k_depth else None
            if to_color is not None:
                cloud = transform(depth_points(depth_to_metres(depth), k_depth), *to_color)
                pixels, front = project(cloud, k_color)
                pts, pixels = cloud[front], pixels[front]
        to_odom = self._tf('odom', color_frame, stamp)
        to_base = self._tf('base_footprint', color_frame, stamp)
        to_map = self._tf('map', color_frame, stamp)

        detections = []
        for (x1, y1, x2, y2), cls, score in zip(boxes, classes, scores):
            obj = {'cls': CLASSES[int(cls)], 'conf': round(float(score), 2),
                   'box': [round(float(x1) / w, 4), round(float(y1) / h, 4),
                           round(float(x2) / w, 4), round(float(y2) / h, 4)]}
            found = locate(pts, pixels, (x1, y1, x2, y2)) if pts is not None else None
            if found is not None and to_odom is not None:
                centre, surface, near = found
                obj['dist'] = round(near, 2)
                obj['width'] = round(float(x2 - x1) * near / k_color[0], 2)
                # Tracking (and the person rings) use the middle of the object;
                # the near side is what the web panel measures distances to.
                odom = transform(centre[None], *to_odom)[0]
                obj['pos'] = (float(odom[0]), float(odom[1]))
                if to_base is not None:
                    b = transform(surface[None], *to_base)[0]
                    obj['base'] = [round(float(b[0]), 2), round(float(b[1]), 2)]
                if to_map is not None:
                    m = transform(surface[None], *to_map)[0]
                    obj['map'] = [round(float(m[0]), 2), round(float(m[1]), 2)]
            detections.append(obj)

        now = time.monotonic()
        tracks = self.tracker.update([d for d in detections if 'pos' in d], now)
        people = [(t['pos'][0], t['pos'][1], t['seen']) for t in tracks.values()
                  if t['cls'] == 'person']
        with self.lock:
            self.people = people
        for d in detections:
            d.pop('pos', None)
        self.objects_pub.publish(String(data=json.dumps(
            {'t': time.time(), 'size': [w, h], 'depth': pts is not None, 'objects': detections})))

    # ---------------- people for the collision monitor ----------------

    def _destination_in_odom(self):
        """(x, y) of the current destination in odom, or None."""
        dest = self.destination
        if dest is None:
            return None
        msg, received = dest
        if time.monotonic() - received > self.destination_max_age:
            return None
        to_odom = self._tf('odom', msg.header.frame_id)
        if to_odom is None:
            return None
        p = msg.pose.position
        x, y, _ = transform(np.array([[p.x, p.y, 0.0]]), *to_odom)[0]
        return float(x), float(y)

    def _publish_cloud(self):
        # In odom, where the people are tracked: the collision monitor moves
        # the points to the robot's pose at its own time.
        now = time.monotonic()
        with self.lock:
            people = [(x, y) for x, y, seen in self.people if now - seen <= self.person_max_age]
        if not self.guard:
            people = []
        exclude = self._destination_in_odom() if people else None
        points = person_rings(people, self.person_radius, RING_POINTS, RING_HEIGHT,
                              exclude, self.goal_exclusion)
        self.cloud_pub.publish(make_cloud(points, 'odom', self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
