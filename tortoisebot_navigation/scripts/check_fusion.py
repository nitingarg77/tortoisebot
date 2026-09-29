#!/usr/bin/env python3
"""Check that /scan_fused is live, correct and actually using the camera.

    ros2 run tortoisebot_navigation check_fusion.py [--seconds 30]

Commands no motion. It only subscribes, so it is safe to run while the robot
is navigating, and safe to run while it is not.

It exists because "the node is running" is not the question. The node publishes
a perfectly good lidar-only scan when the camera has died, when the fan is
stale, when the IMU has stopped and when fusion itself has thrown -- that is
the designed degradation. Every one of those looks identical from outside
unless something measures how often the camera actually changed a beam.

It answers four questions, in the order they can fail:

1. Is /scan_fused publishing, and at the lidar's rate? If it lags /scan, the
   node is the bottleneck.
2. Does it ever differ from /scan? If not, the camera is contributing nothing
   and the node is an elaborate passthrough.
3. Does it ever make a beam *further* away? It must not: fusion takes the
   nearer of the two, so this failing means the rule is broken.
4. Is controller_server's TF buffer advancing? That is the hang this whole
   change exists to remove, and MODULE.md records its signature -- a frozen
   `Transform time` in the tf_help log, where a moving one is ordinary lag.

Exit status is 0 only if 1-3 pass. Question 4 is reported, not asserted: it
needs Nav2 up and a goal in flight, which this tool does not arrange.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Check(Node):

    def __init__(self, raw_topic, fused_topic):
        super().__init__('check_fusion')
        self.raw = {}            # stamp -> ranges, so the comparison is against
        self.raw_count = 0       # the same scan rather than the nearest in time
        self.fused_count = 0
        self.matched = 0
        self.changed = 0         # fused scans where at least one beam differs
        self.beams_changed = 0
        self.beams_unblinded = 0  # lidar had no return, fusion supplied one
        self.beams_pushed_out = 0  # fusion made a beam FURTHER -- must be zero
        self.worst_push = 0.0
        self.first = None
        self.last = None
        self.create_subscription(LaserScan, raw_topic, self._on_raw,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, fused_topic, self._on_fused,
                                 qos_profile_sensor_data)

    def _on_raw(self, msg):
        self.raw_count += 1
        self.raw[round(stamp_of(msg), 6)] = list(msg.ranges)
        if len(self.raw) > 200:                      # bounded, ~17 s at 11.7 Hz
            for k in sorted(self.raw)[:100]:
                del self.raw[k]

    def _on_fused(self, msg):
        now = time.time()
        self.first = self.first or now
        self.last = now
        self.fused_count += 1
        raw = self.raw.get(round(stamp_of(msg), 6))
        if raw is None:
            return
        self.matched += 1
        differed = False
        for a, b in zip(raw, msg.ranges):
            if a == b:
                continue
            differed = True
            self.beams_changed += 1
            valid_a = a == a and msg.range_min <= a <= msg.range_max
            if not valid_a:
                self.beams_unblinded += 1
            elif b > a:
                self.beams_pushed_out += 1
                self.worst_push = max(self.worst_push, b - a)
        if differed:
            self.changed += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    # Defaults assume the empty namespace the launch file uses; pass these if
    # the robot is brought up under one.
    ap.add_argument('--scan', default='/scan')
    ap.add_argument('--fused', default='/scan_fused')
    args = ap.parse_args()

    rclpy.init()
    node = Check(args.scan, args.fused)
    deadline = time.time() + args.seconds
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    span = (node.last - node.first) if node.first and node.last else 0.0
    rate = node.fused_count / span if span > 0.5 else 0.0
    ok = True

    print('\n--- %.0f s ---' % args.seconds)
    print('/scan        %4d messages' % node.raw_count)
    print('/scan_fused  %4d messages, %.1f Hz' % (node.fused_count, rate))

    if node.raw_count == 0:
        print('FAIL  nothing on %s either, so this says nothing about fusion. '
              'The lidar is down.' % args.scan)
        ok = False
    elif node.fused_count == 0:
        print('FAIL  nothing on %s. Is scan_fusion running?' % args.fused)
        print('      ros2 node list | grep scan_fusion')
        ok = False
    elif node.raw_count and node.fused_count < 0.9 * node.raw_count:
        print('FAIL  /scan_fused is dropping scans (%d of %d). The node is the '
              'bottleneck.' % (node.fused_count, node.raw_count))
        ok = False
    else:
        print('ok    publishing at the lidar\'s rate')

    if node.matched == 0:
        print('WARN  no fused scan matched a raw scan by stamp, so nothing '
              'could be compared. The node should be republishing the lidar\'s '
              'own stamp.')
        ok = False
    else:
        pct = 100.0 * node.changed / node.matched
        print('camera changed at least one beam in %d of %d scans (%.0f%%), '
              '%d beams in total' % (node.changed, node.matched, pct,
                                     node.beams_changed))
        print('  of those, %d were beams the lidar saw nothing on -- the '
              'panels' % node.beams_unblinded)
        if node.changed == 0:
            print('FAIL  the camera is contributing nothing. Check '
                  '/floor_scan is publishing and that the node logs '
                  '"camera used in N%%" above zero.')
            ok = False

        if node.beams_pushed_out:
            print('FAIL  %d beams were made FURTHER away (worst %.2f m). '
                  'Fusion must only ever pull a reading nearer; this means '
                  'the rule is broken and the costmap may be clearing real '
                  'obstacles.' % (node.beams_pushed_out, node.worst_push))
            ok = False
        else:
            print('ok    no beam was ever pushed further away')

    print('\nStill to check by eye, with Nav2 up and a goal in flight:')
    print('  ros2 topic echo /rosout | grep -A1 "Transform data too old"')
    print('  A frozen "Transform time" is the hang. A moving one is lag.')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
