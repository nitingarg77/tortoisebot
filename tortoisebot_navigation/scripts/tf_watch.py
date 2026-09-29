#!/usr/bin/env python3
"""Is map->odom frozen on the wire, or only inside controller_server?

    ros2 run tortoisebot_navigation tf_watch.py [--seconds 120]

Commands no motion. It only listens, and the freeze it hunts happens while the
robot is idle, so it needs no goal.

This is the one measurement that separates the two explanations for the hang
recorded in MODULE.md. On 2026-09-29, with `/scan_fused` as the single
observation source, `controller_server` logged 895 `Transform data too old`
lines over 90 s carrying exactly ONE `Transform time` -- a frozen buffer --
while `Data time` advanced normally. That rules the buffer frozen but says
nothing about why:

  (a) Cartographer stopped publishing map->odom, so every listener is stuck
      and the fault is in SLAM; or
  (b) Cartographer kept publishing and only controller_server's listener
      stopped being served, which is what 09-28 concluded.

The two are indistinguishable from controller_server's own logs, and they need
opposite fixes. This process is a THIRD party: it holds its own tf2 buffer with
its own listener, and it also counts raw /tf messages straight off the wire, so
it can tell apart:

  - transform not published    -> raw map->odom messages stop arriving
  - published but not served   -> raw messages keep arriving while this
                                  buffer's newest stamp stops advancing
  - everyone healthy           -> both advance, and the fault is elsewhere

Run it alongside Nav2 and leave it. It prints a line whenever the state
changes, and a verdict at the end.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def stamp_of(t):
    return t.sec + t.nanosec * 1e-9


class Watch(Node):
    """Own buffer, own listener, plus a raw /tf counter."""

    def __init__(self, parent='map', child='odom'):
        super().__init__('tf_watch')
        self.parent, self.child = parent, child
        self.buf = Buffer()
        TransformListener(self.buf, self)

        # Raw /tf, bypassing the buffer entirely. If these keep arriving while
        # the buffer stops advancing, the publisher is fine and the listener
        # is not being served -- which is the whole question.
        self.raw_count = 0
        self.raw_newest = None
        self.create_subscription(TFMessage, '/tf', self._raw,
                                 QoSProfile(depth=100))
        static = QoSProfile(depth=100)
        static.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        static.history = QoSHistoryPolicy.KEEP_LAST
        self.create_subscription(TFMessage, '/tf_static', lambda m: None, static)

    def _raw(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id.lstrip('/') == self.parent and \
                    tr.child_frame_id.lstrip('/') == self.child:
                self.raw_count += 1
                self.raw_newest = stamp_of(tr.header.stamp)

    def buffer_newest(self):
        """Newest map->odom stamp this buffer will serve, or None."""
        try:
            t = self.buf.lookup_transform(self.parent, self.child,
                                          rclpy.time.Time())
            return stamp_of(t.header.stamp)
        except Exception:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--parent', default='map')
    ap.add_argument('--child', default='odom')
    args = ap.parse_args()

    rclpy.init()
    n = Watch(args.parent, args.child)
    print('watching %s -> %s for %.0f s. No motion is commanded.'
          % (args.parent, args.child, args.seconds))

    samples = []                       # (wall, buffer_newest, raw_count)
    last_buf, frozen_since, worst_freeze = None, None, 0.0
    raw_at_freeze = None
    end = time.time() + args.seconds
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.2)
            now = time.time()
            if samples and now - samples[-1][0] < 1.0:
                continue
            b = n.buffer_newest()
            samples.append((now, b, n.raw_count))

            if b is not None and b == last_buf:
                if frozen_since is None:
                    frozen_since, raw_at_freeze = now, n.raw_count
                held = now - frozen_since
                if held > worst_freeze:
                    worst_freeze = held
                if held > 3.0 and int(held) % 5 == 0:
                    print('  FROZEN %5.1f s at stamp %.3f  |  raw /tf '
                          'map->odom since freeze: %d'
                          % (held, b, n.raw_count - raw_at_freeze))
            else:
                if frozen_since is not None and now - frozen_since > 3.0:
                    print('  recovered after %.1f s' % (now - frozen_since))
                frozen_since, raw_at_freeze = None, None
            last_buf = b
    except KeyboardInterrupt:
        pass

    total_raw = n.raw_count
    print('\n--- verdict ---')
    print('raw /tf %s->%s messages : %d in %.0f s (%.1f Hz)'
          % (args.parent, args.child, total_raw, args.seconds,
             total_raw / max(args.seconds, 1e-9)))
    print('longest buffer freeze     : %.1f s' % worst_freeze)

    rc = 0
    if total_raw == 0:
        print('\nNOBODY IS PUBLISHING %s->%s. Not a listener problem: the '
              'transform is absent from the wire. Look at Cartographer '
              '(provide_odom_frame) before anything else.'
              % (args.parent, args.child))
        rc = 1
    elif worst_freeze < 3.0:
        print('\nHealthy here. This buffer tracked the wire throughout. If '
              'controller_server was frozen over the same window, the fault '
              'is inside that process -- its listener is not being served, '
              'which is explanation (b) in MODULE.md.')
    else:
        # Did the wire keep moving while this buffer stood still?
        moved = any(s[2] > samples[i][2] for i, s in enumerate(samples[1:]))
        if moved:
            print('\nThe wire kept delivering while THIS buffer froze for '
                  '%.1f s. A freeze reproduces in an ordinary listener, so it '
                  'is not specific to controller_server: suspect callback '
                  'starvation on this Pi rather than anything Nav2 does.'
                  % worst_freeze)
        else:
            print('\nBoth the wire and this buffer stopped. Cartographer '
                  'stopped publishing -- explanation (a) in MODULE.md, not a '
                  'listener problem at all.')
        rc = 1

    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
