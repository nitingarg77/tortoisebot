#!/usr/bin/env python3
"""Drive a fixed course of Nav2 goals and report what happened to each one.

Written to check the Nav2 TF remap fix (dc8b7cc): remapping /tf on the Nav2
nodes made map->odom go stale partway through a run and cut this course from
10/10 to 1-4/10. One goal still passed, which is how it slipped through, so
the check has to be a long run.

    python3 nav_course.py --laps 2 --timeout 120
    python3 nav_course.py --namespace robot1        # use_namespace:=True

On a real robot the built-in places mean nothing: they are simulator
coordinates. Teach it the real ones by pushing or driving the robot to each
spot, then run the course against that file:

    python3 nav_course.py --record my_room.yaml    # name each spot in turn
    python3 nav_course.py --places my_room.yaml --laps 1

The default places and the QoS settings come from spike_needle_nl/tb_tools.py
and backend.py, both validated against a live stack.
"""

import argparse
import math
import sys
import time

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

PLACES = {
    'kitchen': (0.52, 2.32, 0.0),
    'living_room': (-0.58, 2.32, 1.57),
    'bedroom': (-0.58, -0.58, 3.14),
    'hallway': (0.15, -0.60, 0.0),
    'charging_dock': (0.00, 0.00, 0.0),
}
ORDER = ['kitchen', 'living_room', 'bedroom', 'hallway', 'charging_dock']


def load_places(path):
    """Read {name: [x, y, yaw]} from a YAML file, keeping its order."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    places = {}
    for name, value in data.items():
        if not (isinstance(value, (list, tuple)) and len(value) == 3):
            sys.exit(f'{path}: {name} should be [x, y, yaw], got {value!r}')
        places[name] = tuple(float(v) for v in value)
    if not places:
        sys.exit(f'{path}: no places in it')
    return places, list(places)


STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
          GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


class Course(Node):

    def __init__(self, timeout=120.0, namespace='', places=None):
        super().__init__('nav_course', namespace=namespace)
        self.timeout = timeout
        self.places = places or PLACES
        self.pose = None
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # amcl_pose is TRANSIENT_LOCAL and only published when the estimate
        # updates; a VOLATILE subscriber joining late receives nothing.
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_pose(self, msg):
        self.pose = msg.pose.pose

    def spin_until(self, done, limit):
        end = time.monotonic() + limit
        while rclpy.ok() and not done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return done()

    def goal_for(self, place):
        x, y, yaw = self.places[place]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def run_goal(self, place):
        """Returns (status text, seconds, metres from the target)."""
        started = time.monotonic()
        send = self.nav.send_goal_async(self.goal_for(place))
        if not self.spin_until(lambda: send.done(), 10.0):
            return 'NO RESPONSE TO GOAL', time.monotonic() - started, None
        handle = send.result()
        if not handle.accepted:
            return 'REJECTED', time.monotonic() - started, None

        result = handle.get_result_async()
        if not self.spin_until(lambda: result.done(), self.timeout):
            handle.cancel_goal_async()
            self.spin_until(lambda: False, 2.0)
            return 'TIMED OUT', time.monotonic() - started, self.error(place)
        status = STATUS.get(result.result().status,
                            f'STATUS {result.result().status}')
        return status, time.monotonic() - started, self.error(place)

    def error(self, place):
        if self.pose is None:
            return None
        x, y, _ = self.places[place]
        return math.hypot(self.pose.position.x - x, self.pose.position.y - y)

    def current(self):
        """Where the robot thinks it is: (x, y, yaw), or None."""
        if self.pose is None:
            return None
        q = self.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        return self.pose.position.x, self.pose.position.y, yaw


def record(node, path):
    """Teach the course real coordinates: drive the robot to each spot and
    name it. Nav2 need not be running, but AMCL must be, since the positions
    come from /amcl_pose."""
    places = {}
    print(f'Recording places into {path}.')
    print('Drive or push the robot to a spot, type a name, press Enter.')
    print('Press Enter on an empty name when done.\n')
    while True:
        try:
            name = input('name for this spot (empty to finish): ').strip()
        except EOFError:
            break
        if not name:
            break
        node.spin_until(lambda: False, 0.5)   # take the latest pose
        here = node.current()
        if here is None:
            print('  no /amcl_pose yet: is AMCL running and localised?')
            continue
        places[name] = [round(v, 3) for v in here]
        print(f'  {name}: x={here[0]:.2f} y={here[1]:.2f} yaw={here[2]:.2f}')
    if not places:
        print('nothing recorded')
        return
    with open(path, 'w') as f:
        yaml.safe_dump(places, f, sort_keys=False, default_flow_style=None)
    print(f'\nwrote {len(places)} places to {path}')
    print(f'run the course with:  python3 nav_course.py --places {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--timeout', type=float, default=120.0,
                    help='seconds allowed per goal')
    ap.add_argument('--namespace', default='',
                    help='robot namespace, e.g. robot1, when the stack was '
                         'launched with use_namespace:=True')
    ap.add_argument('--places', metavar='FILE',
                    help='YAML of {name: [x, y, yaw]}; the default places are '
                         'simulator coordinates and are wrong on a real robot')
    ap.add_argument('--record', metavar='FILE',
                    help='drive the robot around and write its positions to '
                         'FILE instead of running a course')
    args = ap.parse_args()

    places, order = (load_places(args.places) if args.places
                     else (PLACES, ORDER))

    rclpy.init()
    node = Course(args.timeout, args.namespace, places)

    if args.record:
        if not node.spin_until(lambda: node.pose is not None, 30.0):
            sys.exit('no /amcl_pose: start navigation and localise first')
        record(node, args.record)
        node.destroy_node()
        rclpy.shutdown()
        return

    print('waiting for navigate_to_pose...', flush=True)
    if not node.nav.wait_for_server(timeout_sec=60.0):
        sys.exit('no navigate_to_pose action server: is Nav2 up?')
    if not node.spin_until(lambda: node.pose is not None, 30.0):
        print('WARNING: no /amcl_pose yet; distance errors will be missing')

    rows = []
    for lap in range(args.laps):
        for place in order:
            n = len(rows) + 1
            print(f'[{n:2}] -> {place} ...', end=' ', flush=True)
            status, secs, err = node.run_goal(place)
            rows.append((n, place, status, secs, err))
            print(f'{status} in {secs:5.1f}s'
                  + (f', {err:.2f} m from target' if err is not None else ''),
                  flush=True)

    ok = sum(1 for r in rows if r[2] == 'SUCCEEDED')
    print(f'\n{"#":>2}  {"goal":<14} {"result":<20} {"time":>7}  {"error":>7}')
    for n, place, status, secs, err in rows:
        e = f'{err:.2f} m' if err is not None else '-'
        print(f'{n:2}  {place:<14} {status:<20} {secs:6.1f}s  {e:>7}')
    print(f'\n{ok}/{len(rows)} goals reached')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok == len(rows) else 1)


if __name__ == '__main__':
    main()
