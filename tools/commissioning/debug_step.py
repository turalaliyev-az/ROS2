"""
Speed step with the firmware's speed-loop diagnostics (rover service STOPPED).

Like speed_step.py, but switches on the 0xAA 0x56 diagnostics frames
(0xCC 0x36) and prints, per wheel, what the loop sees and does: target and
measured ticks/s, output PWM, break-away push, integral, ms since the last tick.
No lidar protection; the robot drives straight at most MAX_DIST.
Usage: debug_step.py <m/s, + = robot forward> [max distance m]
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
DRIVE = yaml.safe_load(open(pathlib.Path.home() / '.rover' / 'robot.yaml'))['drive']
SIGN = -1.0 if DRIVE.get('reversed') else 1.0
M_PER_TICK = {'l': math.pi * DRIVE['wheel_diameter'] / DRIVE['ticks_per_rev_left'],
              'r': math.pi * DRIVE['wheel_diameter'] / DRIVE['ticks_per_rev_right']}
ACCEL, DECEL = 0.5, 1.0
SPEED = float(sys.argv[1])
MAX_DIST = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
LOG = pathlib.Path(__file__).parent / 'logs'

ser = serial.Serial(PORT, 115200, timeout=0.05)
lock = threading.Lock()
enc = []     # (t, l, r)
dbg = []     # (t, 12 values)
running = True


def reader():
    buf = bytearray()
    while running:
        try:
            buf.extend(ser.read(max(1, ser.in_waiting)))
        except (serial.SerialException, OSError, TypeError):
            return
        while True:
            i = buf.find(b'\xaa')
            if i < 0:
                buf.clear()
                break
            del buf[:i]
            if len(buf) < 29:
                break
            f = bytes(buf[:29])
            chk = 0
            for b in f[2:28]:
                chk ^= b
            if f[1] in (0x55, 0x56) and chk == f[28]:
                now = time.monotonic()
                with lock:
                    if f[1] == 0x55:
                        enc.append((now,) + struct.unpack_from('<ii', f, 20))
                    else:
                        dbg.append((now,) + struct.unpack_from('<12h', f, 2))
                del buf[:29]
            else:
                del buf[:1]


def send(sync1, a, b):
    payload = struct.pack('<hh', int(round(a)), int(round(b)))
    chk = 0
    for x in payload:
        chk ^= x
    ser.write(bytes([0xCC, sync1]) + payload + bytes([chk]))


def slew(cur, tgt, dt):
    up = cur * tgt >= 0 and abs(tgt) > abs(cur)
    step = (ACCEL if up else DECEL) * dt
    return cur + max(-step, min(step, tgt - cur))


threading.Thread(target=reader, daemon=True).start()
try:
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        send(0x34, 0, 0)
        time.sleep(0.02)
    if not enc:
        sys.exit('no telemetry (is the rover service stopped?)')
    send(0x36, 0, 0)
    last_dbg_req = time.monotonic()
    _, l0, r0 = enc[-1]
    v, t0, t_stop = 0.0, time.monotonic(), None
    while True:
        now = time.monotonic()
        if now - last_dbg_req > 5:
            send(0x36, 0, 0)
            last_dbg_req = now
        _, l, r = enc[-1]
        dist = ((l - l0) * M_PER_TICK['l'] + (r - r0) * M_PER_TICK['r']) / 2 * SIGN
        driving = t_stop is None and abs(dist) < MAX_DIST and now - t0 < MAX_DIST / abs(SPEED) * 3 + 2
        if not driving and t_stop is None:
            t_stop = now
        v = slew(v, SPEED if driving else 0.0, 0.02)
        dv = SIGN * v
        send(0x34, dv / M_PER_TICK['l'], dv / M_PER_TICK['r'])
        if t_stop is not None and now - t_stop > 1.2:
            break
        time.sleep(0.02)
finally:
    for _ in range(10):
        send(0x34, 0, 0)
        time.sleep(0.02)
    running = False
    time.sleep(0.1)
    ser.close()

print(f'>>> hədəf {SPEED:+.2f} m/s (drive çərçivəsində işarə {SIGN:+.0f}); STOP {t_stop - t0:.2f} s')
print('  t    | SOL: hədəf ölçülən  PWM  təkan  inteq  ms | SAĞ: hədəf ölçülən  PWM  təkan  inteq  ms')
nxt = 0.0
rows = [d for d in dbg if d[0] >= t0]
for d in rows:
    t = d[0] - t0
    if t >= nxt:
        tl, tr, sl, sr, ol, orr, bl, br, il, ir, ml, mr = d[1:]
        print(f'{t:5.2f} |      {tl:5d} {sl:7d} {ol:5d} {bl:5d} {il:6d} {ml:4d} |      '
              f'{tr:5d} {sr:7d} {orr:5d} {br:5d} {ir:6d} {mr:4d}' + ('  ← STOP' if abs(t - (t_stop - t0)) < 0.06 else ''))
        nxt += 0.1
LOG.mkdir(exist_ok=True)
with open(LOG / 'debug_steps.jsonl', 'a') as f:
    f.write(json.dumps({'time': time.strftime('%F %T'), 'speed': SPEED, 'stop_s': t_stop - t0,
                        'debug': [(round(d[0] - t0, 3),) + d[1:] for d in rows]}) + '\n')
