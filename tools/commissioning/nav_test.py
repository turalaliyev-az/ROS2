"""
Navigation acceptance test: pick goals on the map, drive there through the web
server (same path as the panel) and back, and report each leg.

Goals: A = 2-3 m away in open floor with a clear line of sight; B = 5-9 m away
with a planned path (checked with Nav2's planner first); then back to the
start pose. Every goal keeps >= CLEARANCE from mapped walls.
Rover service running in navigation mode, robot localized. Usage: nav_test.py
"""
import asyncio
import json
import math
import time

import aiohttp
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage

CLEARANCE = 0.7
LEG_TIMEOUT = 240.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def plan_goals():
    rclpy.init()
    node = rclpy.create_node('nav_test_planner')
    got = {}
    node.create_subscription(OccupancyGrid, 'map', lambda m: got.setdefault('map', m), LATCHED)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                             lambda m: got.__setitem__('amcl', m), LATCHED)
    end = time.monotonic() + 15
    while time.monotonic() < end and not {'map', 'amcl'} <= got.keys():
        rclpy.spin_once(node, timeout_sec=0.1)
    m = got['map']
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    dist = ndimage.distance_transform_edt(~(grid >= 65)) * res
    dist[grid < 0] = 0.0          # unknown space is no place to go
    p, q = got['amcl'].pose.pose.position, got['amcl'].pose.pose.orientation
    start = (p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def clearance_at(x, y):
        i, j = int((x - ox) / res), int((y - oy) / res)
        if 0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]:
            return float(dist[j, i])
        return 0.0

    def line_clearance(a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
        return min(clearance_at(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                   for k in range(n + 1))

    jj, ii = np.nonzero(dist >= CLEARANCE)
    xs, ys = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
    rng = np.hypot(xs - start[0], ys - start[1])

    # A: near, straight line of sight
    near = np.nonzero((rng >= 2.0) & (rng <= 3.0))[0][::7]
    scored = sorted(((min(dist[jj[k], ii[k]], line_clearance(start, (xs[k], ys[k]))), xs[k], ys[k])
                     for k in near), reverse=True)
    a = scored[0] if scored and scored[0][0] >= 0.45 else None

    # B: further, with a planned path
    planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    planner.wait_for_server(timeout_sec=10.0)
    far = np.nonzero((rng >= 5.0) & (rng <= 9.0))[0]
    far = far[np.argsort(-dist[jj[far], ii[far]])][::25][:20]
    b = None
    for k in far:
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x, goal.goal.pose.position.y = float(xs[k]), float(ys[k])
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = 'GridBased'
        goal.use_start = False
        f = planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        h = f.result()
        if h is None or not h.accepted:
            continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=10.0)
        path = rf.result().result.path.poses if rf.result() else []
        if len(path) < 2:
            continue
        length = sum(math.hypot(q2.pose.position.x - q1.pose.position.x,
                                q2.pose.position.y - q1.pose.position.y) for q1, q2 in zip(path, path[1:]))
        if length <= 1.6 * float(rng[k]):
            b = (float(dist[jj[k], ii[k]]), float(xs[k]), float(ys[k]), length)
            break
    node.destroy_node()
    rclpy.shutdown()
    return start, a, b


async def drive(ws, name, x, y, yaw):
    await ws.send_str(json.dumps({'cmd': 'goto', 'x': x, 'y': y, 'yaw': yaw, 'label': name}))
    t0 = time.monotonic()
    started, result, pose = False, None, None
    print(f'>>> {name}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)', flush=True)
    while time.monotonic() - t0 < LEG_TIMEOUT:
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=2.0)
        except asyncio.TimeoutError:
            continue
        if msg.type != aiohttp.WSMsgType.TEXT:
            continue
        data = json.loads(msg.data)
        if data.get('t') == 'event':
            print(f"    [{data['level']}] {data['text']}", flush=True)
            if data['level'] == 'error' and ('qəbul edilmədi' in data['text'] or 'mümkün olmadı' in data['text']):
                result = 'UĞURSUZ'
            if 'Çatdı' in data['text'] and name in data['text']:
                result = 'ÇATDI'
        elif data.get('t') == 'state':
            nav, pose = data.get('nav') or {}, data.get('pose') or pose
            if nav.get('active'):
                started = True
            elif started and result:
                break
        if result == 'UĞURSUZ':
            break
    else:
        await ws.send_str(json.dumps({'cmd': 'stop'}))
        result = 'VAXT BİTDİ (STOP göndərildi)'
    dt = time.monotonic() - t0
    err = math.hypot(pose['x'] - x, pose['y'] - y) if pose else float('nan')
    yaw_err = math.degrees(math.remainder(pose['yaw'] - yaw, 2 * math.pi)) if pose else float('nan')
    print(f'    NƏTİCƏ: {result}, {dt:.0f} s, mövqe xətası {err * 100:.0f} sm, istiqamət xətası {yaw_err:+.0f}°',
          flush=True)
    return result, dt, err, yaw_err


async def main():
    start, a, b = plan_goals()
    print(f'başlanğıc: ({start[0]:.2f}, {start[1]:.2f}, {math.degrees(start[2]):.0f}°)')
    print(f'A: {"tapılmadı" if a is None else f"({a[1]:.2f}, {a[2]:.2f}), boşluq {a[0]:.2f} m"}')
    print(f'B: {"tapılmadı" if b is None else f"({b[1]:.2f}, {b[2]:.2f}), boşluq {b[0]:.2f} m, plan {b[3]:.1f} m"}',
          flush=True)
    legs = []
    prev = start
    if a:
        legs.append(('Test A', a[1], a[2], math.atan2(a[2] - prev[1], a[1] - prev[0])))
        prev = (a[1], a[2])
    if b:
        legs.append(('Test B', b[1], b[2], math.atan2(b[2] - prev[1], b[1] - prev[0])))
    legs.append(('Başlanğıc', start[0], start[1], start[2]))
    results = []
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect('http://127.0.0.1:8080/ws') as ws:
            for leg in legs:
                results.append((leg[0],) + await drive(ws, *leg))
                await asyncio.sleep(2.0)
    print('\nYEKUN:')
    for name, result, dt, err, yerr in results:
        print(f'  {name:10} {result:28} {dt:4.0f} s  mövqe xətası {err * 100:3.0f} sm  istiqamət {yerr:+4.0f}°')


asyncio.run(main())
