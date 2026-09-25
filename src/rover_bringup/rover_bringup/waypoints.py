"""
Drive the robot through map points in order (Nav2 FollowWaypoints).

    ros2 run rover_bringup waypoints 1.2,-0.5 2.0,1.0 0.5,2.3,90
    ros2 run rover_bringup waypoints --loops 2 1.2,-0.5 2.0,1.0

Each point is x,y (metres, map frame) with an optional heading in degrees.
Without a heading the robot arrives facing the way it was travelling.
Ctrl+C cancels the route and stops the robot.
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints


def parse_args(argv):
    loops = 0
    points = []
    i = 0
    while i < len(argv):
        if argv[i] == '--loops':
            loops = int(argv[i + 1])
            i += 2
            continue
        # Parsed by hand rather than argparse: map coordinates are often
        # negative, and argparse takes "-1.5,2" for an unknown option.
        values = [float(v) for v in argv[i].split(',')]
        if len(values) not in (2, 3):
            raise ValueError(f'"{argv[i]}" should be x,y or x,y,heading_deg')
        points.append(values)
        i += 1
    return loops, points


def build_poses(points, start_xy, stamp):
    poses = []
    prev = start_xy
    for i, point in enumerate(points):
        x, y = point[0], point[1]
        if len(point) == 3:
            yaw = math.radians(point[2])
        elif prev is not None and (x, y) != prev:
            yaw = math.atan2(y - prev[1], x - prev[0])
        elif i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        else:
            yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        poses.append(pose)
        prev = (x, y)
    return poses


def current_robot_xy(node, timeout_s=3.0):
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    found = []
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda msg: found.append(msg), latched)
    end = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while not found and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not found:
        return None
    p = found[-1].pose.pose.position
    return (p.x, p.y)


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    try:
        loops, points = parse_args(argv)
    except (ValueError, IndexError) as exc:
        print(f'Xəta: {exc}\n{__doc__}')
        return 1
    if not points:
        print(__doc__)
        return 1

    # rclpy's own SIGINT handler would shut the context down on Ctrl+C before
    # the cancel request could be sent, leaving the robot driving the route.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('waypoints_cli')
    client = ActionClient(node, FollowWaypoints, 'follow_waypoints')
    if not client.wait_for_server(timeout_sec=10.0):
        print('Nav2 tapılmadı: əvvəlcə nav2.launch.py işə salın və mövqe verin.')
        rclpy.try_shutdown()
        return 1

    goal = FollowWaypoints.Goal()
    goal.poses = build_poses(points, current_robot_xy(node), node.get_clock().now().to_msg())
    goal.number_of_loops = loops

    total = len(points)
    shown = [-1]

    def on_feedback(msg):
        idx = msg.feedback.current_waypoint
        if idx != shown[0] and idx < total:
            shown[0] = idx
            x, y = points[idx][0], points[idx][1]
            print(f'-> nöqtə {idx + 1}/{total}: ({x:.2f}, {y:.2f})', flush=True)

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('Marşrut qəbul edilmədi (Nav2 aktivdirmi, mövqe verilibmi?).')
        rclpy.try_shutdown()
        return 1
    print(f'Marşrut başladı: {total} nöqtə' + (f', {loops} əlavə dövrə' if loops else ''))

    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        print('\nLəğv edilir, robot dayanır...')
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        rclpy.try_shutdown()
        return 1

    result = result_future.result().result
    missed = [m.index + 1 for m in result.missed_waypoints]
    if missed:
        print(f'Bitdi, amma bu nöqtələrə çatmaq olmadı: {missed}')
    else:
        print('Bütün nöqtələrə çatıldı.')
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
