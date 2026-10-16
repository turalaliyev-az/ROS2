"""
Check the encoder decoding before the speed loop ever uses it (rover service
STOPPED, calibration firmware with the 0xCC 0x35 raw-PWM frame).

A short fixed-PWM turn in place, no closed loop: the wheel ticks converted with
the given ticks/turn must give the same rotation as the ESP32's gyro (within
the ~10 % odometry calibration error found earlier), and each wheel must count
in the direction it was driven.
Usage: verify_decoding.py <ticks/turn left> <ticks/turn right> [pwm] [seconds]
"""
import math
import pathlib
import struct
import sys
import threading
import time

import serial
import yaml

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C4E000865-if00'
ticks = {'l': float(sys.argv[1]), 'r': float(sys.argv[2])}
PWM = float(sys.argv[3]) if len(sys.argv) > 3 else 380
SECONDS = float(sys.argv[4]) if len(sys.argv) > 4 else 0.6
drive = yaml.safe_load(open(pathlib.Path.home() / '.rover' / 'robot.yaml'))['drive']
m_per_tick = {k: math.pi * drive['wheel_diameter'] / v for k, v in ticks.items()}
SEP = drive['track']

ser = serial.Serial(PORT, 115200, timeout=0.05)
lock = threading.Lock()
frames = []   # (t, enc_l, enc_r, gz deg/s)
running = True


def reader():
    buf = bytearray()
    while running:
        try:
            buf.extend(ser.read(max(1, ser.in_waiting)))
        except (serial.SerialException, OSError, TypeError):
            return
        while True:
            i = buf.find(b'\xaa\x55')
            if i < 0:
                del buf[:-1]
                break
            del buf[:i]
            if len(buf) < 29:
                break
            f = bytes(buf[:29])
            chk = 0
            for b in f[2:28]:
                chk ^= b
            if chk == f[28]:
                gz = struct.unpack_from('<h', f, 12)[0] / 10.0
                l, r = struct.unpack_from('<ii', f, 20)
                with lock:
                    frames.append((time.monotonic(), l, r, gz))
                del buf[:29]
            else:
                del buf[:2]


def pwm(left, right):
    payload = struct.pack('<hh', int(left), int(right))
    chk = 0
    for b in payload:
        chk ^= b
    ser.write(b'\xcc\x35' + payload + bytes([chk]))


threading.Thread(target=reader, daemon=True).start()
try:
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        pwm(0, 0)
        time.sleep(0.02)
    if not frames:
        sys.exit('no telemetry (is the rover service stopped?)')
    t0 = time.monotonic()
    while time.monotonic() - t0 < SECONDS:
        pwm(-PWM, PWM)          # drive frame: left back, right forward = turn left
        time.sleep(0.02)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        pwm(0, 0)
        time.sleep(0.02)
finally:
    for _ in range(10):
        pwm(0, 0)
        time.sleep(0.02)
    running = False
    time.sleep(0.1)
    ser.close()

with lock:
    seg = [f for f in frames if f[0] >= t0 - 0.05]
start, last = seg[0], seg[-1]
dl, dr = last[1] - start[1], last[2] - start[2]
gyro = sum((b[0] - a[0]) * a[3] for a, b in zip(seg, seg[1:]))
enc = math.degrees((dr * m_per_tick['r'] - dl * m_per_tick['l']) / SEP)
print(f'impulslar: sol {dl:+d} (geri sürülüb → mənfi olmalı), sağ {dr:+d} (irəli → müsbət olmalı)')
print(f'dönmə: enkoder {enc:+.1f}°, giroskop {gyro:+.1f}°')
ok_sign = dl < 0 < dr
ratio = enc / gyro if abs(gyro) > 3 else float('nan')
ok_ratio = 0.7 < ratio < 1.15
ok_balance = min(abs(dl), abs(dr)) > 0.5 * max(abs(dl), abs(dr))
print(f'enkoder/giroskop = {ratio:.3f} (gözlənilən ~0.89)')
print('NƏTİCƏ: ' + ('DEKODLAMA DÜZGÜNDÜR' if ok_sign and ok_ratio and ok_balance else
                   'XƏTA: ' + ', '.join(m for ok, m in ((ok_sign, 'işarələr səhv'),
                                                        (ok_ratio, 'nisbət səhv'),
                                                        (ok_balance, 'təkərlər balanssız')) if not ok)))
