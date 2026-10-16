"""
Speed step test straight on the ESP32 (rover service STOPPED): command a
robot-frame speed like motor_bridge would (same slew), log both encoders at
~100 Hz, and report the speed profile, overshoot, stalls and stopping.

No lidar protection: the robot drives straight at most MAX_DIST. The
firmware's 200 ms watchdog stops the motors if this program dies.
Usage: speed_step.py <m/s, + = robot forward> [max distance m]
"""
import json
import math
import pathlib
import struct
import sys
import threading
import time

import serial
import yaml

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C4E000865-if00'
profile = yaml.safe_load(open(pathlib.Path.home() / '.rover' / 'robot.yaml'))
DRIVE = profile['drive']
SIGN = -1.0 if DRIVE.get('reversed') else 1.0
M_PER_TICK = {'l': math.pi * DRIVE['wheel_diameter'] / DRIVE['ticks_per_rev_left'],
              'r': math.pi * DRIVE['wheel_diameter'] / DRIVE['ticks_per_rev_right']}
ACCEL, DECEL = 0.5, 1.0          # m/s^2, as motor_bridge (profile.ACCEL / DECEL)
SPEED = float(sys.argv[1])
MAX_DIST = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
LOG = pathlib.Path(__file__).parent / 'logs'


class Esp:
    def __init__(self):
        self.ser = serial.Serial(PORT, 115200, timeout=0.05)
        self.lock = threading.Lock()
        self.hist = []          # (t, enc_l, enc_r)
        self.running = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = bytearray()
        while self.running:
            buf.extend(self.ser.read(max(1, self.ser.in_waiting)))
            while True:
                i = buf.find(b'\xaa\x55')
                if i < 0:
                    del buf[:-1]
                    break
                del buf[:i]
                if len(buf) < 29:
                    break
                frame = bytes(buf[:29])
                chk = 0
                for b in frame[2:28]:
                    chk ^= b
                if chk == frame[28]:
                    l, r = struct.unpack_from('<ii', frame, 20)
                    with self.lock:
                        self.hist.append((time.monotonic(), l, r))
                    del buf[:29]
                else:
                    del buf[:2]

    def targets(self, tl, tr):
        payload = struct.pack('<hh', int(round(tl)), int(round(tr)))
        chk = 0
        for b in payload:
            chk ^= b
        self.ser.write(b'\xcc\x34' + payload + bytes([chk]))

    def latest(self):
        with self.lock:
            return self.hist[-1] if self.hist else None

    def close(self):
        for _ in range(10):
            self.targets(0, 0)
            time.sleep(0.02)
        self.running = False
        self.ser.close()


def slew(cur, tgt, dt):
    up = cur * tgt >= 0 and abs(tgt) > abs(cur)
    step = (ACCEL if up else DECEL) * dt
    return cur + max(-step, min(step, tgt - cur))


esp = Esp()
deadline = time.monotonic() + 3
while esp.latest() is None:
    esp.targets(0, 0)
    time.sleep(0.05)
    if time.monotonic() > deadline:
        sys.exit('no telemetry from the ESP32 (is the rover service still running?)')
for _ in range(25):
    esp.targets(0, 0)
    time.sleep(0.02)
_, l0, r0 = esp.latest()
v = 0.0
t0 = time.monotonic()
t_stop = None
try:
    while True:
        now = time.monotonic()
        _, l, r = esp.latest()
        dist = ((l - l0) * M_PER_TICK['l'] + (r - r0) * M_PER_TICK['r']) / 2 * SIGN
        driving = t_stop is None and abs(dist) < MAX_DIST and now - t0 < MAX_DIST / abs(SPEED) * 3 + 2
        if not driving and t_stop is None:
            t_stop = now
        v = slew(v, SPEED if driving else 0.0, 0.02)
        drive_v = SIGN * v
        esp.targets(drive_v / M_PER_TICK['l'], drive_v / M_PER_TICK['r'])
        if t_stop is not None and now - t_stop > 1.5:
            break
        time.sleep(0.02)
finally:
    esp.close()

with esp.lock:
    hist = [h for h in esp.hist if h[0] >= t0]


def speeds(window=0.05):
    out = []
    j = 0
    for i, (t, l, r) in enumerate(hist):
        while hist[j][0] < t - window:
            j += 1
        dt = t - hist[j][0]
        if dt > 0.02:
            out.append((t - t0, SIGN * (l - hist[j][1]) * M_PER_TICK['l'] / dt,
                        SIGN * (r - hist[j][2]) * M_PER_TICK['r'] / dt))
    return out


sp = speeds()
ts = t_stop - t0
print(f'>>> hədəf {SPEED:+.2f} m/s, {MAX_DIST} m; STOP {ts:.2f} s-də')
print(' zaman  SOL m/s  SAĞ m/s')
nxt = 0.0
for t, vl, vr in sp:
    if t >= nxt:
        print(f'{t:5.2f}  {vl:+6.3f}  {vr:+6.3f}' + ('  ← STOP' if abs(t - ts) < 0.05 else ''))
        nxt += 0.1
drive = [s for s in sp if 1.0 <= s[0] <= ts]
for name, k in (('SOL', 1), ('SAĞ', 2)):
    vals = [s[k] for s in drive]
    if vals:
        still = sum(1 for x in vals if abs(x) < 0.01) / len(vals)
        print(f'{name}: 1 s-dən STOP-a qədər orta {sum(vals) / len(vals):+.3f}, '
              f'maks {max(vals, key=abs):+.3f}, dayanıqlı {100 * still:.0f}%')
moving_after = max((s[0] - ts for s in sp if s[0] > ts and (abs(s[1]) > 0.01 or abs(s[2]) > 0.01)),
                   default=0.0)
_, l, r = hist[-1]
_, ls, rs = min(hist, key=lambda h: abs(h[0] - t_stop))
coast = ((l - ls) * M_PER_TICK['l'] + (r - rs) * M_PER_TICK['r']) / 2 * SIGN
print(f'dayanma: STOP-dan sonra {moving_after:.2f} s, {coast:+.3f} m')
LOG.mkdir(exist_ok=True)
with open(LOG / 'speed_steps.jsonl', 'a') as f:
    f.write(json.dumps({'time': time.strftime('%F %T'), 'speed': SPEED, 'dist': MAX_DIST,
                        'stop_s': ts, 'profile': [(round(t, 3), round(a, 3), round(b, 3)) for t, a, b in sp]})
            + '\n')
