"""
Rover control panel: web server + operating-mode manager.

Serves a phone-friendly page on port 8080. Access needs the robot's key, which
is embedded in the QR code shown on the robot's own screen (/qr, localhost
only), so other devices on the same Wi-Fi can't drive the robot.
"""
import asyncio
import glob
import io
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np
import qrcode
import qrcode.image.svg
import rclpy
from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from rover_web.processes import ProcessManager
from rover_web.ros_side import MANUAL_MAX_TURN, RosSide
from rover_web.store import Store, valid_name

STATIC_DIR = os.path.join(get_package_share_directory('rover_web'), 'static')
SEED_MAP = os.path.join(get_package_share_directory('rover_bringup'), 'maps',
                        'restaurant_map_v2.yaml')
COOKIE = 'rover_key'
NAV_SPEED_RANGE = (0.08, 0.3)
MANUAL_SPEED_RANGE = (0.05, 0.3)

QR_PAGE = '''<!doctype html>
<html lang="az"><head><meta charset="utf-8"><title>Rover</title>
<style>
body{margin:0;height:100vh;display:flex;flex-direction:column;align-items:center;
  justify-content:center;background:#0f172a;color:#e2e8f0;font-family:sans-serif;
  text-align:center;overflow:hidden;cursor:none}
img{width:min(58vh,80vw);background:#fff;border-radius:16px;padding:12px}
h1{margin:0 0 16px;font-size:2.2em}
#urls{opacity:.85;font-size:1.4em;margin:14px 0 4px}
#status{font-size:1.5em;line-height:1.8}
.on{color:#22c55e}.off{color:#ef4444}
</style></head><body>
<h1 id="title">Sistem başladılır…</h1>
<img id="qr" hidden alt="">
<div id="urls"></div>
<div id="status"></div>
<script>
const MODES = {navigation: 'Naviqasiya', mapping: 'Xəritələmə', idle: 'Gözləmə'};
const STATES = {ready: ['hazır', '#22c55e'], starting: ['başladılır…', '#f59e0b'],
                waiting: ['motor və lidar gözlənilir…', '#f59e0b'],
                error: ['xəta', '#ef4444'], idle: ['', '#94a3b8']};
const SENSORS = [['esp32', 'Motor'], ['lidar', 'Lidar'], ['camera', 'Kamera']];
const $ = id => document.getElementById(id);
let shownUrl = null;
async function poll() {
  try {
    const s = await (await fetch('/qr.json', {cache: 'no-store'})).json();
    const url = s.urls[0] || '';
    if (url !== shownUrl) {
      shownUrl = url;
      $('qr').hidden = !url;
      if (url) $('qr').src = '/qr.svg?u=' + encodeURIComponent(url);
    }
    $('title').textContent = url ? 'Robotu idarə etmək üçün skan edin'
                                 : 'Şəbəkə yoxdur — robotu Wi-Fi-a qoşun';
    $('urls').textContent = s.urls.join('  ·  ');
    const [stateText, color] = STATES[s.mode_state] || ['', '#94a3b8'];
    $('status').innerHTML =
      `<b style="color:${color}">${MODES[s.mode] || s.mode}${stateText ? ' · ' + stateText : ''}</b><br>` +
      SENSORS.map(([k, label]) => `<span class="${s.sensors[k] ? 'on' : 'off'}">●</span> ${label}`)
             .join(' &nbsp; ');
  } catch (e) {
    shownUrl = null;
    $('qr').hidden = true;
    $('title').textContent = 'Sistem başladılır…';
    $('urls').textContent = '';
    $('status').textContent = '';
  }
}
poll();
setInterval(poll, 3000);
</script></body></html>
'''


def lan_ips():
    ips = []
    try:
        # No packet is sent: connect() on UDP only picks the outgoing interface.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 80))
            ips.append(s.getsockname()[0])
    except OSError:
        pass
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
        ips += [ip for ip in out.split() if ':' not in ip and ip not in ips]
    except (OSError, subprocess.SubprocessError):
        pass
    return ips or ['127.0.0.1']


def render_map_png(msg):
    info = msg.info
    grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    img = np.full(grid.shape, 205, dtype=np.uint8)
    known = grid >= 0
    img[known] = (254 - np.clip(grid[known], 0, 100) * 2.54).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(img[::-1], 'L').save(buf, format='PNG')  # row 0 = top = max y
    return buf.getvalue()


def _turbo_lut():
    """Google's Turbo colormap (polynomial fit): 0 = dark blue ... 255 = dark red."""
    x = np.linspace(0.0, 1.0, 256)
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (
        -152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (
        4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (
        -89.90310912 + x * 27.34824973))))
    return (np.clip(np.stack([r, g, b], axis=1), 0.0, 1.0) * 255).astype(np.uint8)


TURBO = _turbo_lut()
DEPTH_NEAR_MM, DEPTH_FAR_MM = 300.0, 6000.0


def encode_camera(msg, kind, max_width=640):
    """sensor_msgs/Image -> JPEG bytes; depth is coloured red (near) to blue (far)."""
    h, w = msg.height, msg.width
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if kind == 'depth':
        depth = data.view('>u2' if msg.is_bigendian else '<u2').reshape(h, msg.step // 2)[:, :w]
        x = np.clip((depth.astype(np.float32) - DEPTH_NEAR_MM) / (DEPTH_FAR_MM - DEPTH_NEAR_MM),
                    0.0, 1.0)
        rgb = TURBO[(255 - x * 225).astype(np.uint8)]  # far end stops at blue, not near-black
        rgb[depth == 0] = 0  # no reading
    else:
        channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
        rgb = data.reshape(h, msg.step)[:, :w * channels].reshape(h, w, channels)[:, :, :3]
        if msg.encoding.startswith('bgr'):
            rgb = rgb[:, :, ::-1]
    img = Image.fromarray(np.ascontiguousarray(rgb), 'RGB')
    if w > max_width:
        img = img.resize((max_width, round(h * max_width / w)))
    buf = io.BytesIO()
    img.save(buf, format='JPEG', quality=70)
    return buf.getvalue()


def read_battery():
    """The PC's own battery: {'percent', 'plugged'}, or None without one."""
    override = os.environ.get('ROVER_BATTERY_DIR')  # tests point this at fake files
    for d in [override] if override else sorted(glob.glob('/sys/class/power_supply/BAT*')):
        try:
            with open(os.path.join(d, 'capacity')) as f:
                percent = int(f.read().strip())
            with open(os.path.join(d, 'status')) as f:
                status = f.read().strip()
        except (OSError, ValueError):
            continue
        # "Not charging"/"Full"/"Unknown" also mean the charger is connected.
        return {'percent': percent, 'plugged': status != 'Discharging'}
    return None


def clamp(value, lo, hi):
    return max(lo, min(hi, float(value)))


class Controller:
    """Owns the operating mode and turns browser commands into robot actions."""

    def __init__(self, ros, store, procs, port):
        self.ros = ros
        self.store = store
        self.procs = procs
        self.port = port
        self.mode = 'idle'           # idle | navigation | mapping
        self.mode_state = 'idle'     # waiting (for sensors) | starting | ready | error
        self.explorer_on = False
        self.loc_confirmed = False
        self.external_nav = False  # Nav2 run by someone else (simulator)
        self.mode_task = None
        self.joy = (0.0, 0.0)
        self.joy_time = 0.0
        self.driving = False
        self.saved_pose = None
        self.clients = set()
        self._map_png_cache = (None, b'')
        self._camera_cache = {}      # kind -> (Image msg, jpeg)
        self.explore_when_ready = False
        self.battery = read_battery()
        self.auto_home_level = None  # battery % at the last automatic trip home
        ros.nav_speed = store.settings['nav_speed']

    # ---------------- modes ----------------

    async def set_mode(self, mode, explore=False):
        """explore: in mapping mode, start autonomous exploration as soon as it is ready."""
        if self.mode_task:
            self.mode_task.cancel()
        self.external_nav = False
        self.ros.cancel_everything()
        self.explorer_on = False
        self.explore_when_ready = explore and mode == 'mapping'
        self.mode, self.mode_state = mode, 'waiting' if mode != 'idle' else 'idle'
        await asyncio.to_thread(self.procs.stop, 'explorer')
        await asyncio.to_thread(self.procs.stop, 'navigation')
        await asyncio.to_thread(self.procs.stop, 'mapping')
        with self.ros.lock:
            self.ros.map_msg = None
            self.ros.map_version += 1
            self.ros.amcl = None
        if mode == 'navigation':
            active = self.store.settings['active_map']
            if not active:
                self.mode, self.mode_state = 'idle', 'idle'
                self.ros.event('error', 'Xəritə yoxdur: əvvəlcə xəritələmə edin')
                return
            self.mode_task = asyncio.create_task(self._bring_up_navigation(active))
        elif mode == 'mapping':
            self.mode_task = asyncio.create_task(self._bring_up_mapping())

    async def _wait_for_base(self):
        """Hold off until the motor board and the lidar are talking.

        Nav2 and slam_toolbox give up if odometry or scans are missing while
        they start, and the robot's electronics may be powered after the PC.
        """
        while not all(self.ros.sensors()[k] for k in ('esp32', 'lidar')):
            await asyncio.sleep(0.5)
        self.mode_state = 'starting'

    async def _start_and_wait(self, name, cmd, ready, attempts=2, timeout=60.0):
        """Start a mode's processes and wait until ready() says so.

        Nav2's lifecycle bring-up occasionally loses a race with service
        discovery under load and aborts; a second start almost always works.
        ready(first_poll) is polled every 0.5 s; first_poll is True right
        after each (re)start.
        """
        for attempt in range(attempts):
            if attempt:
                self.ros.event('warn', 'Başlama alınmadı, yenidən cəhd edilir…')
                await asyncio.to_thread(self.procs.stop, name)
            self.procs.start(name, cmd)
            first = True
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and self.procs.running(name):
                if ready(first):
                    return True
                first = False
                await asyncio.sleep(0.5)
        return False

    async def _bring_up_navigation(self, map_name):
        await self._wait_for_base()
        pose = self.store.last_pose(map_name)
        self.loc_confirmed = pose is not None
        if pose is None:
            # AMCL needs *some* pose before Nav2 finishes starting (the global
            # costmap gives up after ~20 s without map->odom); the operator
            # is asked to set the real one.
            pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        seen = {'amcl': 0}

        def ready(first):
            if first:
                seen['amcl'] = self.ros.amcl_count
            if self.ros.amcl_count == seen['amcl']:
                self.ros.set_initial_pose(pose['x'], pose['y'], pose['yaw'],
                                          0.3 if self.loc_confirmed else 1.0)
                return False
            return self.ros.nav_client.server_is_ready()

        cmd = ['ros2', 'launch', 'rover_bringup', 'nav2.launch.py',
               f'map:={self.store.map_yaml(map_name)}']
        if await self._start_and_wait('navigation', cmd, ready):
            self.mode_state = 'ready'
            self.ros.event('info', 'Naviqasiya hazırdır' if self.loc_confirmed else
                           'Naviqasiya hazırdır, amma mövqe məlum deyil: xəritədə təyin edin')
            return
        self.mode_state = 'error'
        self.ros.event('error', 'Naviqasiya başlamadı. "Naviqasiyanı yenidən başlat" basın. '
                       'Loq: ~/.rover/logs/navigation.log')

    async def _bring_up_mapping(self):
        await self._wait_for_base()

        def ready(_first):
            return self.ros.map_msg is not None and self.ros.nav_client.server_is_ready()

        cmd = ['ros2', 'launch', 'rover_bringup', 'mapping.launch.py']
        if await self._start_and_wait('mapping', cmd, ready):
            self.mode_state = 'ready'
            if self.explore_when_ready:
                self.explore_when_ready = False
                await asyncio.to_thread(self.set_explorer, True)
            else:
                self.ros.event('info', 'Xəritələmə hazırdır: robotu sürün və ya '
                               'avtonom kəşfi başladın')
            return
        self.mode_state = 'error'
        self.explore_when_ready = False
        self.ros.event('error', 'Xəritələmə başlamadı. Loq: ~/.rover/logs/mapping.log')

    def set_explorer(self, on):
        if on and not (self.mode == 'mapping' and self.mode_state == 'ready'):
            self.ros.event('error', 'Avtonom kəşf yalnız xəritələmə rejimində işləyir')
            return
        self.ros.cancel()
        self.explorer_on = on
        if on:
            self.ros.explore_status = 'starting'
            self.procs.start('explorer', ['ros2', 'run', 'rover_bringup', 'explorer'])
            self.ros.event('info', 'Avtonom kəşf başladı')
        else:
            # The explorer's own goal would keep the robot driving; cancel it
            # both before (stop now) and after (nothing sent while exiting).
            self.ros.cancel_everything()
            self.procs.stop('explorer')
            self.ros.cancel_everything()
            self.ros.explore_status = ''

    async def save_map(self, name, activate):
        if self.mode != 'mapping' or self.ros.map_msg is None:
            self.ros.event('error', 'Xəritəni yalnız xəritələmə rejimində saxlamaq olar')
            return False
        if not valid_name(name):
            self.ros.event('error', 'Ad yalnız hərf, rəqəm, _ və - ola bilər')
            return False
        pose = self.ros.robot_pose()
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f',
               self.store.map_prefix(name), '--ros-args', '-p', 'save_map_timeout:=10.0']
        result = await asyncio.to_thread(subprocess.run, cmd, capture_output=True, text=True,
                                         timeout=40)
        if result.returncode != 0 or name not in self.store.list_maps():
            self.ros.event('error', 'Xəritəni saxlamaq alınmadı')
            return False
        if pose:
            # The robot is exactly here in the new map; navigation can start
            # already localized instead of asking for a position.
            self.store.save_last_pose(name, pose['x'], pose['y'], pose['yaw'])
        self.ros.event('info', f'Xəritə saxlanıldı: {name}')
        if activate:
            self.store.set('active_map', name)
            await self.set_mode('navigation')
        return True

    # ---------------- home (charging spot) ----------------

    def home(self):
        return self.store.home(self.store.settings['active_map'])

    def at_home(self):
        home, pose = self.home(), self.ros.robot_pose()
        return bool(home and pose and math.hypot(pose['x'] - home['x'], pose['y'] - home['y']) < 0.3)

    def go_home(self):
        if self.mode != 'navigation' or self.mode_state != 'ready':
            self.ros.event('error', 'Evə qayıtmaq üçün naviqasiya rejimi hazır olmalıdır')
            return False
        home = self.home()
        if home is None:
            self.ros.event('error', 'Ev nöqtəsi təyin edilməyib (Ayarlar → Ev və şarj)')
            return False
        return self.ros.goto(home['x'], home['y'], home['yaw'], 'Evə qayıdır', kind='home')

    def _check_auto_home(self):
        """Send the robot home when the PC battery runs low (once per 5 % step)."""
        b, s = self.battery, self.store.settings
        if b is None or b['plugged']:
            self.auto_home_level = None  # re-arm for the next discharge
            return
        if not s['auto_home'] or b['percent'] > s['auto_home_level']:
            return
        # The operator may send it out again on purpose; ask again only after
        # another 5 % is gone.
        if self.auto_home_level is not None and b['percent'] > self.auto_home_level - 5:
            return
        if self.mode != 'navigation' or self.mode_state != 'ready':
            return  # tried again once navigation is up
        self.auto_home_level = b['percent']
        with self.ros.lock:
            going_home = self.ros.nav['active'] and self.ros.nav['kind'] == 'home'
        if going_home or self.at_home():
            return
        self.ros.event('warn', f'Batareya {b["percent"]}%: robot evə qayıdır')
        self.go_home()

    async def activate_map(self, name):
        if name not in self.store.list_maps():
            return
        self.store.set('active_map', name)
        self.ros.event('info', f'Aktiv xəritə: {name}')
        await self.set_mode('navigation')

    # ---------------- periodic work ----------------

    async def joystick_loop(self):
        while True:
            fresh = time.monotonic() - self.joy_time < 0.4
            if fresh:
                v = self.joy[1] * self.store.settings['manual_speed']
                w = -self.joy[0] * MANUAL_MAX_TURN
                self.ros.drive(v, w)
                self.driving = True
            elif self.driving:
                for _ in range(3):
                    self.ros.drive(0.0, 0.0)
                self.driving = False
            await asyncio.sleep(0.05)

    async def housekeeping_loop(self):
        while True:
            await asyncio.sleep(2.0)
            self.battery = read_battery()
            self._check_auto_home()
            if self.explorer_on and self.ros.explore_status == 'done':
                self.explorer_on = False
                await asyncio.to_thread(self.procs.stop, 'explorer')
                name = datetime.now().strftime('xerite_%Y%m%d_%H%M')
                self.ros.event('info', 'Avtonom xəritələmə bitdi, saxlanılır...')
                # The robot is back at its start, which save_map records, so
                # navigation on the new map starts already localized.
                await self.save_map(name, activate=True)
            if self.mode == 'mapping' and self.explorer_on and not self.procs.running('explorer'):
                self.explorer_on = False
                self.ros.event('warn', 'Avtonom kəşf dayandı')
            if self.mode in ('navigation', 'mapping') and self.mode_state == 'ready' and \
                    not self.external_nav and not self.procs.running(self.mode):
                self.mode_state = 'error'
                self.ros.event('error', 'Naviqasiya prosesi dayandı. "Yenidən başlat" basın.')
            self._remember_pose()

    def _remember_pose(self):
        if self.mode != 'navigation' or self.mode_state != 'ready':
            return
        with self.ros.lock:
            amcl = dict(self.ros.amcl) if self.ros.amcl else None
        if not amcl or amcl['std_xy'] > 0.5:
            return
        last = self.saved_pose
        if last and math.hypot(amcl['x'] - last[0], amcl['y'] - last[1]) < 0.05 and \
                abs(amcl['yaw'] - last[2]) < 0.05:
            return
        self.store.save_last_pose(self.store.settings['active_map'],
                                  amcl['x'], amcl['y'], amcl['yaw'])
        self.saved_pose = (amcl['x'], amcl['y'], amcl['yaw'])

    # ---------------- state out ----------------

    def state(self):
        ros = self.ros
        with ros.lock:
            nav = dict(ros.nav)
            amcl = dict(ros.amcl) if ros.amcl else None
            map_msg = ros.map_msg
            version = ros.map_version
        map_meta = None
        if map_msg is not None:
            info = map_msg.info
            map_meta = {'version': version, 'width': info.width, 'height': info.height,
                        'resolution': info.resolution,
                        'ox': info.origin.position.x, 'oy': info.origin.position.y}
        pose = ros.robot_pose()
        active = self.store.settings['active_map']
        return {
            't': 'state',
            'mode': self.mode, 'mode_state': self.mode_state,
            'pose': pose,
            'loc': ({'std_xy': round(amcl['std_xy'], 2),
                     'std_yaw': round(math.degrees(amcl['std_yaw']), 1)} if amcl else None),
            'loc_confirmed': self.loc_confirmed,
            'nav': nav,
            'explorer': {'on': self.explorer_on, 'status': ros.explore_status},
            'sensors': ros.sensors(),
            'velocity': [round(ros.velocity[0], 2), round(ros.velocity[1], 2)],
            'speed': {'nav': self.store.settings['nav_speed'],
                      'manual': self.store.settings['manual_speed'],
                      'nav_range': NAV_SPEED_RANGE, 'manual_range': MANUAL_SPEED_RANGE},
            'map': map_meta,
            'active_map': active,
            'maps': self.store.list_maps(),
            'tables': self.store.tables(active) if self.mode != 'mapping' else {},
            'home': self.home() if self.mode != 'mapping' else None,
            'battery': self.battery,
            'auto_home': {'on': self.store.settings['auto_home'],
                          'level': self.store.settings['auto_home_level']},
            'explore_pending': self.explore_when_ready,
            'scan': ros.scan_points('map'),
        }

    def camera_jpeg(self, kind):
        self.ros.want_camera(kind)
        msg = self.ros.camera_msgs.get(kind)
        if msg is None:
            return None
        cached = self._camera_cache.get(kind)
        if cached is None or cached[0] is not msg:
            cached = (msg, encode_camera(msg, kind))
            self._camera_cache[kind] = cached
        return cached[1]

    def map_png(self):
        with self.ros.lock:
            msg, version = self.ros.map_msg, self.ros.map_version
        if msg is None:
            return None
        if self._map_png_cache[0] != version:
            self._map_png_cache = (version, render_map_png(msg))
        return self._map_png_cache[1]

    async def broadcast_loop(self):
        while True:
            await asyncio.sleep(0.2)
            if not self.clients:
                self.ros.events.clear()
                continue
            messages = [json.dumps(self.state())]
            while self.ros.events:
                level, text = self.ros.events.popleft()
                messages.append(json.dumps({'t': 'event', 'level': level, 'text': text}))
            for ws in list(self.clients):
                try:
                    for m in messages:
                        await ws.send_str(m)
                except (ConnectionResetError, RuntimeError):
                    self.clients.discard(ws)

    # ---------------- commands from the browser ----------------

    async def handle(self, cmd):
        name = cmd.get('cmd')
        ros = self.ros
        tables_map = self.store.settings['active_map']

        if name == 'joy':
            x, y = clamp(cmd.get('x', 0), -1, 1), clamp(cmd.get('y', 0), -1, 1)
            if abs(x) > 0.05 or abs(y) > 0.05:
                # Operator takes over: autonomy yields immediately.
                if ros.nav_active():
                    ros.cancel()
                    ros.event('warn', 'Manual idarə: avtonom hərəkət dayandırıldı')
                if self.explorer_on:
                    await asyncio.to_thread(self.set_explorer, False)
            self.joy, self.joy_time = (x, y), time.monotonic()
        elif name == 'stop':
            self.joy, self.joy_time = (0.0, 0.0), 0.0
            ros.cancel_everything()
            for _ in range(3):
                ros.drive(0.0, 0.0)
            ros.event('warn', 'STOP')
            if self.explorer_on:
                await asyncio.to_thread(self.set_explorer, False)
        elif name == 'goto':
            x, y = float(cmd['x']), float(cmd['y'])
            yaw = float(cmd['yaw']) if cmd.get('yaw') is not None else ros.goal_yaw(x, y)
            ros.goto(x, y, yaw, cmd.get('label') or f'({x:.2f}, {y:.2f})')
        elif name == 'goto_table':
            t = self.store.tables(tables_map).get(cmd.get('name'))
            if t:
                ros.goto(t['x'], t['y'], t['yaw'], f'Masa {cmd["name"]}')
        elif name == 'route':
            points = [(float(p[0]), float(p[1]), None if len(p) < 3 or p[2] is None else float(p[2]))
                      for p in cmd.get('points', [])]
            if points:
                ros.route(points, int(cmd.get('loops', 0)), f'Marşrut ({len(points)} nöqtə)')
        elif name == 'route_tables':
            tables = self.store.tables(tables_map)
            names = [n for n in cmd.get('names', []) if n in tables]
            points = [(tables[n]['x'], tables[n]['y'], tables[n]['yaw']) for n in names]
            if points:
                ros.route(points, int(cmd.get('loops', 0)), 'Masalar: ' + ', '.join(names))
        elif name == 'set_pose':
            if self.mode != 'navigation':
                ros.event('error', 'Mövqe yalnız naviqasiya rejimində təyin olunur')
                return
            ros.cancel()
            ros.set_initial_pose(float(cmd['x']), float(cmd['y']), float(cmd['yaw']))
            self.loc_confirmed = True
            ros.event('info', 'Mövqe təyin edildi')
        elif name == 'relocalize':
            if self.mode == 'navigation':
                ros.relocalize()
                self.loc_confirmed = True
        elif name == 'speed':
            if 'nav' in cmd:
                v = round(clamp(cmd['nav'], *NAV_SPEED_RANGE), 3)
                self.store.set('nav_speed', v)
                ros.nav_speed = v
            if 'manual' in cmd:
                self.store.set('manual_speed', round(clamp(cmd['manual'], *MANUAL_SPEED_RANGE), 3))
        elif name == 'mode':
            if cmd.get('mode') in ('navigation', 'mapping', 'idle'):
                await self.set_mode(cmd['mode'], explore=bool(cmd.get('explore')))
                if self.explore_when_ready:
                    ros.event('info', 'Avtonom xəritələmə başlayır: robot hazır olan kimi '
                              'özü hərəkət edəcək')
        elif name == 'home_go':
            self.go_home()
        elif name in ('home_set', 'home_set_here'):
            if self.mode != 'navigation' or not tables_map:
                ros.event('error', 'Ev nöqtəsi naviqasiya rejimində təyin olunur')
                return
            if name == 'home_set_here':
                pose = ros.robot_pose()
                if pose is None:
                    ros.event('error', 'Robotun mövqeyi məlum deyil')
                    return
                x, y, yaw = pose['x'], pose['y'], pose['yaw']
            else:
                x, y = float(cmd['x']), float(cmd['y'])
                yaw = float(cmd['yaw']) if cmd.get('yaw') is not None else \
                    math.radians(float(cmd.get('yaw_deg') or 0.0))
            self.store.set_home(tables_map, x, y, yaw)
            ros.event('info', f'Ev nöqtəsi yadda saxlanıldı ({x:.2f}, {y:.2f})')
        elif name == 'auto_home':
            if 'on' in cmd:
                self.store.set('auto_home', bool(cmd['on']))
            if 'level' in cmd:
                self.store.set('auto_home_level', int(clamp(cmd['level'], 10, 60)))
            self.auto_home_level = None
        elif name == 'restart':
            await self.set_mode(self.mode if self.mode != 'idle' else 'navigation')
        elif name == 'explore':
            await asyncio.to_thread(self.set_explorer, bool(cmd.get('on')))
        elif name == 'save_map':
            await self.save_map(cmd.get('name', ''), bool(cmd.get('activate')))
        elif name == 'activate_map':
            await self.activate_map(cmd.get('name', ''))
        elif name in ('table_set', 'table_here'):
            tname = str(cmd.get('name', '')).strip()
            if not tname or len(tname) > 30 or not tables_map:
                ros.event('error', 'Masa adı düzgün deyil')
                return
            if name == 'table_here':
                pose = ros.robot_pose()
                if pose is None:
                    ros.event('error', 'Robotun mövqeyi məlum deyil')
                    return
                x, y, yaw = pose['x'], pose['y'], pose['yaw']
            else:
                x, y = float(cmd['x']), float(cmd['y'])
                yaw = math.radians(float(cmd.get('yaw_deg') or 0.0))
            self.store.set_table(tables_map, tname, x, y, yaw)
            ros.event('info', f'Masa yadda saxlanıldı: {tname} ({x:.2f}, {y:.2f})')
        elif name == 'table_delete':
            tname = cmd.get('name')
            if tname in self.store.tables(tables_map):
                self.store.delete_table(tables_map, tname)
                ros.event('info', f'Masa silindi: {tname}')


# ---------------- HTTP / WebSocket ----------------

def build_app(ctrl):
    key = ctrl.store.settings['access_key']

    def is_local(request):
        return request.remote in ('127.0.0.1', '::1')

    @web.middleware
    async def auth(request, handler):
        if request.path in ('/qr', '/qr.svg', '/qr.json') and is_local(request):
            return await handler(request)
        if request.query.get('key') == key:
            resp = web.HTTPFound(request.path)
            resp.set_cookie(COOKIE, key, max_age=365 * 24 * 3600, httponly=True, samesite='Strict')
            raise resp
        if request.cookies.get(COOKIE) == key or is_local(request):
            return await handler(request)
        return web.Response(status=401, content_type='text/html', text=(
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<body style="font-family:sans-serif;padding:2em;text-align:center">'
            '<h2>Giriş yoxdur</h2><p>Robotun ekranındakı QR kodu telefonla skan edin.</p>'))

    async def index(_request):
        return web.FileResponse(os.path.join(STATIC_DIR, 'index.html'),
                                headers={'Cache-Control': 'no-cache'})

    async def map_png(_request):
        png = await asyncio.to_thread(ctrl.map_png)
        if png is None:
            raise web.HTTPNotFound()
        return web.Response(body=png, content_type='image/png',
                            headers={'Cache-Control': 'no-store'})

    async def camera(request):
        kind = request.query.get('kind', 'color')
        if kind not in ('color', 'depth'):
            raise web.HTTPBadRequest()
        jpeg = await asyncio.to_thread(ctrl.camera_jpeg, kind)
        if jpeg is None:
            # First request after idle: the subscription is being set up.
            return web.Response(status=503, headers={'Retry-After': '1'})
        return web.Response(body=jpeg, content_type='image/jpeg',
                            headers={'Cache-Control': 'no-store'})

    def qr_url():
        return f'http://{lan_ips()[0]}:{ctrl.port}/?key={key}'

    async def qr_svg(_request):
        img = qrcode.make(qr_url(), image_factory=qrcode.image.svg.SvgPathImage, border=2)
        return web.Response(body=img.to_string(), content_type='image/svg+xml')

    async def qr_status(_request):
        return web.json_response({
            'urls': [f'http://{ip}:{ctrl.port}' for ip in lan_ips() if not ip.startswith('127.')],
            'mode': ctrl.mode, 'mode_state': ctrl.mode_state, 'sensors': ctrl.ros.sensors(),
        })

    async def qr_page(_request):
        # The robot's own full-screen display. It polls /qr.json rather than
        # reloading, so it rides out server restarts and follows the IP when
        # Wi-Fi comes up late or changes.
        return web.Response(content_type='text/html', text=QR_PAGE)

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        ctrl.clients.add(ws)
        await ws.send_str(json.dumps(ctrl.state()))
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    await ctrl.handle(json.loads(msg.data))
                except (KeyError, ValueError, TypeError) as exc:
                    ctrl.ros.event('error', f'Yanlış əmr: {exc}')
        finally:
            ctrl.clients.discard(ws)
            # A phone that drops off Wi-Fi mid-drive must not leave the robot
            # rolling on its last joystick input.
            ctrl.joy_time = 0.0
        return ws

    app = web.Application(middlewares=[auth])
    app.router.add_get('/', index)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/map.png', map_png)
    app.router.add_get('/api/camera.jpg', camera)
    app.router.add_get('/qr', qr_page)
    app.router.add_get('/qr.svg', qr_svg)
    app.router.add_get('/qr.json', qr_status)
    app.router.add_static('/static/', STATIC_DIR)
    app['qr_url'] = qr_url
    return app


def main():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    ros = RosSide()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros)
    threading.Thread(target=executor.spin, daemon=True).start()

    store = Store(SEED_MAP)
    procs = ProcessManager(str(store.logs_dir), ros.get_logger())
    port = ros.get_parameter('port').value
    ctrl = Controller(ros, store, procs, port)
    app = build_app(ctrl)

    async def on_startup(_app):
        for coro in (ctrl.joystick_loop(), ctrl.broadcast_loop(), ctrl.housekeeping_loop()):
            asyncio.create_task(coro)

        start_mode = ros.get_parameter('start_mode').value

        async def start_default_mode():
            await asyncio.sleep(1.0)  # let ROS discovery settle; set_mode waits for the sensors
            if ctrl.mode_task is not None:
                return  # an operator already picked a mode; don't undo it
            if start_mode == 'navigation' and not store.settings['active_map']:
                await ctrl.set_mode('idle')
            elif start_mode in ('navigation', 'mapping', 'idle'):
                await ctrl.set_mode(start_mode)
            else:
                ctrl.mode, ctrl.mode_state = 'navigation', 'ready'
                ctrl.external_nav = True
        asyncio.create_task(start_default_mode())

        qr = qrcode.QRCode(border=1)
        qr.add_data(app['qr_url']())
        out = io.StringIO()
        qr.print_ascii(out=out)
        ros.get_logger().info(f'Control panel: {app["qr_url"]()}\n{out.getvalue()}')

    async def on_cleanup(_app):
        for _ in range(3):
            ros.drive(0.0, 0.0)
        await asyncio.to_thread(procs.stop_all)
        executor.shutdown(timeout_sec=2.0)
        ros.destroy_node()
        rclpy.try_shutdown()

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    web.run_app(app, host='0.0.0.0', port=port, print=None)


if __name__ == '__main__':
    main()
