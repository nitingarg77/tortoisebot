"""Send ONE short NavigateToPose goal and report honestly what happened.

Commands motion, so per CLAUDE.md section 2 it carries all three:

1. Clearance guard: refuses to send if the FUSED scan (not the lidar alone)
   shows less than `need` metres in the direction of travel, and keeps
   checking while driving, cancelling the goal if clearance collapses.
2. Did-it-actually-move check: map->base_link before and after, cross-checked
   against the lidar's nearest-ahead. Not a single lidar cone -- that is what
   reported negative progress during the 6.8 m overshoot.
3. Stop on every exit path: the goal is cancelled in `finally`, including on
   exception, timeout and KeyboardInterrupt.

It also watches for the false-success signature: Nav2 reporting the goal
reached while the robot has not moved, which is what a frozen TF buffer plus
isGoalReached() discarding transformPose's return value produces.

READ THIS BEFORE RUNNING IT OVER SSH
------------------------------------
**Killing the ssh that launched this does not stop the robot.** Without a TTY,
ssh does not reliably signal the remote process, and even if it did, Nav2 holds
the goal -- not this script. On 2026-09-29 an interrupted run kept driving for
the full timeout because of exactly that. The timeout is therefore derived from
the distance rather than being a flat constant: it is the leash, and it is the
only thing that ends the run if nobody is watching.

To stop it by hand, cancel the goal rather than killing the script:

    ros2 action send_goal --help      # or, bluntly:
    ros2 lifecycle set /controller_server deactivate
"""
import math, sys, time, threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tortoisebot_navigation.scan_fusion import Fan

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
NEED = DIST + 0.35          # clearance required ahead, goal plus a margin

# The leash, derived from the distance rather than fixed. This chassis manages
# about 0.20 m/s flat out and rather less under DWB, so 0.08 m/s is a pessimistic
# average including the turn onto heading; the constant covers planning, the
# initial rotation and one recovery. A flat 90 s was used once for a 0.80 m goal
# that needs about 6 s, and the robot drove for the whole of it unattended.
TIMEOUT = min(DIST / 0.08 + 15.0, 120.0)


class Runner(Node):
    def __init__(self):
        super().__init__('one_goal')
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.lock = threading.Lock()
        self.fused = None
        self.raw = None
        self.create_subscription(LaserScan, '/scan_fused',
                                 self._fused, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 self._raw, qos_profile_sensor_data)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _fused(self, m):
        with self.lock:
            self.fused = Fan.from_msg(m)

    def _raw(self, m):
        with self.lock:
            self.raw = Fan.from_msg(m)

    def ahead(self, which='fused', half=15.0):
        with self.lock:
            f = self.fused if which == 'fused' else self.raw
        if f is None:
            return None
        v = [r for i, r in enumerate(f.ranges) if f.valid(r) and
             abs((math.degrees(f.bearing(i)) + 180) % 360 - 180) <= half]
        return min(v) if v else None

    def pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link',
                                          rclpy.time.Time()).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        return t.translation.x, t.translation.y, yaw


def spin_for(node, seconds):
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    rclpy.init()
    n = Runner()
    handle = None
    try:
        spin_for(n, 6)
        start = n.pose()
        if start is None:
            print('FAIL  no map->base_link transform; not sending a goal')
            return 1
        cf, cr = n.ahead('fused'), n.ahead('raw')
        print('start pose  x=%.3f y=%.3f yaw=%.1f deg' %
              (start[0], start[1], math.degrees(start[2])))
        print('clearance   fused %s  raw %s  (need %.2f m)' %
              ('%.2f' % cf if cf else 'none', '%.2f' % cr if cr else 'none', NEED))

        # --- guard 1 -----------------------------------------------------
        if cf is None or cf < NEED:
            print('REFUSED  only %s m clear ahead on the fused scan; a %.2f m '
                  'goal needs %.2f m.' % ('%.2f' % cf if cf else 'no return',
                                          DIST, NEED))
            return 1

        gx = start[0] + DIST * math.cos(start[2])
        gy = start[1] + DIST * math.sin(start[2])
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = n.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = gx, gy
        g.pose.orientation.z = math.sin(start[2] / 2)
        g.pose.orientation.w = math.cos(start[2] / 2)

        if not n.ac.wait_for_server(timeout_sec=15.0):
            print('FAIL  navigate_to_pose action server never appeared')
            return 1
        print('goal        x=%.3f y=%.3f  (%.2f m ahead)' % (gx, gy, DIST))

        sent = time.time()
        fut = n.ac.send_goal_async(NavigateToPose.Goal(pose=g))
        while rclpy.ok() and not fut.done() and time.time() - sent < 15:
            rclpy.spin_once(n, timeout_sec=0.1)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print('FAIL  goal rejected')
            return 1
        print('accepted after %.2f s' % (time.time() - sent))

        res_fut = handle.get_result_async()
        t0 = time.time()
        moved_max = 0.0
        while rclpy.ok() and not res_fut.done() and time.time() - t0 < TIMEOUT:
            rclpy.spin_once(n, timeout_sec=0.1)
            p = n.pose()
            if p:
                moved_max = max(moved_max, math.hypot(p[0] - start[0],
                                                      p[1] - start[1]))
            # --- guard 1, while driving ---
            c = n.ahead('fused')
            if c is not None and c < 0.30:
                print('GUARD  only %.2f m ahead, cancelling' % c)
                handle.cancel_goal_async()
                spin_for(n, 3)
                break

        elapsed = time.time() - t0
        end_pose = n.pose()
        travelled = (math.hypot(end_pose[0] - start[0], end_pose[1] - start[1])
                     if end_pose else float('nan'))

        print('--- result ---')
        if res_fut.done():
            code = res_fut.result().status
            names = {2: 'CANCELED', 4: 'SUCCEEDED', 5: 'ABORTED', 6: 'CANCELED'}
            print('status      %s (%d) after %.1f s' %
                  (names.get(code, '?'), code, elapsed))
        else:
            print('status      NO RESULT after %.1f s (timed out)' % elapsed)
        print('end pose    x=%.3f y=%.3f yaw=%.1f deg'
              % (end_pose[0], end_pose[1], math.degrees(end_pose[2]))
              if end_pose else 'end pose    LOST')
        print('travelled   %.3f m  (max excursion %.3f m, asked for %.2f)'
              % (travelled, moved_max, DIST))
        print('clearance   fused %s  raw %s (was %.2f / %.2f)'
              % ('%.2f' % (n.ahead('fused') or float('nan')),
                 '%.2f' % (n.ahead('raw') or float('nan')), cf, cr))

        # --- guard 2: the false-success signature ---
        if res_fut.done() and res_fut.result().status == 4 and travelled < 0.15:
            print('\n*** FALSE SUCCESS: Nav2 reported the goal reached after '
                  '%.1f s having moved %.3f m. This is the frozen-TF signature '
                  '-- the hang is NOT cured. ***' % (elapsed, travelled))
            return 2
        return 0
    finally:
        # --- guard 3: stop on every exit path ---
        try:
            if handle is not None:
                handle.cancel_goal_async()
                spin_for(n, 2)
        except Exception:
            pass
        try:
            n.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
