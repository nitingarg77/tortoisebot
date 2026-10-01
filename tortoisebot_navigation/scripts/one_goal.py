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
from action_msgs.srv import CancelGoal
import argparse

from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener
from tortoisebot_navigation.places import drive_verdict, yaw_from_quaternion
from tortoisebot_navigation.scan_fusion import Fan, wrap

_ap = argparse.ArgumentParser()
_ap.add_argument('dist', nargs='?', type=float, default=0.8,
                 help='metres straight ahead (default 0.8)')
# Rotate in place instead of driving. The TF freeze does not need translation
# -- isGoalReached() looks up map->odom on every control cycle of any goal --
# and +N then -N brings the robot back to where it started, so a comparison
# can be repeated from one spot instead of walking the robot across the room.
_ap.add_argument('--turn', type=float, default=None, metavar='DEG',
                 help='rotate in place by DEG instead of driving')
ARGS = _ap.parse_args()
TURN = math.radians(ARGS.turn) if ARGS.turn is not None else None
DIST = 0.0 if TURN is not None else ARGS.dist
NEED = DIST + 0.35          # clearance required ahead, goal plus a margin
# Rotating sweeps the footprint (0.1 m radius) in every direction, so the
# guard for a turn is the nearest return anywhere, not just ahead.
TURN_NEED = 0.35
TURN_ABORT = 0.20
# general_goal_checker's xy_goal_tolerance in nav2_params_robot.yaml. A success
# may stop this far short, so the false-success check allows for it. A goal
# shorter than this (plus slack) cannot be told apart from not moving.
XY_TOL = 0.15

# The leash, derived from the distance rather than fixed. This chassis manages
# about 0.20 m/s flat out and rather less under DWB, so 0.08 m/s is a pessimistic
# average including the turn onto heading; the constant covers planning, the
# initial rotation and one recovery. A flat 90 s was used once for a 0.80 m goal
# that needs about 6 s, and the robot drove for the whole of it unattended.
TIMEOUT = min(DIST / 0.08 + 15.0, 120.0)
if TURN is not None:
    # 0.25 rad/s is a pessimistic average for DWB's in-place rotation here.
    TIMEOUT = min(abs(TURN) / 0.25 + 15.0, 60.0)


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
        # The BNO055's fused yaw: the did-it-turn check. Deliberately not TF,
        # because TF is the thing under test and may be frozen.
        self.imu_yaw = None
        self.create_subscription(Imu, '/imu', self._imu,
                                 qos_profile_sensor_data)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _fused(self, m):
        with self.lock:
            self.fused = Fan.from_msg(m)

    def _raw(self, m):
        with self.lock:
            self.raw = Fan.from_msg(m)

    def _imu(self, m):
        q = m.orientation
        with self.lock:
            self.imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def nearest(self):
        """Nearest valid return in any direction on the fused scan."""
        with self.lock:
            f = self.fused
        if f is None:
            return None
        v = [r for r in f.ranges if f.valid(r)]
        return min(v) if v else None

    def ahead(self, which='fused', half=15.0):
        with self.lock:
            f = self.fused if which == 'fused' else self.raw
        if f is None:
            return None
        v = [r for i, r in enumerate(f.ranges) if f.valid(r) and
             abs((math.degrees(f.bearing(i)) + 180) % 360 - 180) <= half]
        return min(v) if v else None

    def ahead_median(self, half=5.0):
        """Median raw-lidar range within +-half deg of straight ahead.

        The second witness for a straight drive (CLAUDE.md section 2): a
        median over several beams, raw lidar so a camera false return cannot
        move it. Meaningless if the robot turned, so drives only.
        """
        with self.lock:
            f = self.raw
        if f is None:
            return None
        v = sorted(r for i, r in enumerate(f.ranges) if f.valid(r) and
                   abs((math.degrees(f.bearing(i)) + 180) % 360 - 180) <= half)
        return v[len(v) // 2] if len(v) >= 3 else None

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
    sent = None
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

        imu0 = n.imu_yaw
        med0 = n.ahead_median()
        # --- guard 1 -----------------------------------------------------
        if TURN is not None:
            near = n.nearest()
            print('clearance   nearest anywhere %s (need %.2f m to turn)'
                  % ('%.2f' % near if near else 'none', TURN_NEED))
            if near is None or near < TURN_NEED:
                print('REFUSED  something within %s m; not turning.'
                      % ('%.2f' % near if near else '?'))
                return 1
            if imu0 is None:
                print('REFUSED  no /imu, so a turn could not be verified.')
                return 1
        elif cf is None or cf < NEED:
            print('REFUSED  only %s m clear ahead on the fused scan; a %.2f m '
                  'goal needs %.2f m.' % ('%.2f' % cf if cf else 'no return',
                                          DIST, NEED))
            return 1
        if TURN is None and DIST <= XY_TOL + 0.05:
            print('NOTE  a %.2f m goal is inside the goal tolerance; success '
                  'will not show whether TF is live.' % DIST)

        gx = start[0] + DIST * math.cos(start[2])
        gy = start[1] + DIST * math.sin(start[2])
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = n.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = gx, gy
        gyaw = start[2] if TURN is None else wrap(start[2] + TURN)
        g.pose.orientation.z = math.sin(gyaw / 2)
        g.pose.orientation.w = math.cos(gyaw / 2)

        if not n.ac.wait_for_server(timeout_sec=15.0):
            print('FAIL  navigate_to_pose action server never appeared')
            return 1
        if TURN is None:
            print('goal        x=%.3f y=%.3f  (%.2f m ahead)' % (gx, gy, DIST))
        else:
            print('goal        turn %+.0f deg in place, to yaw %.1f deg'
                  % (math.degrees(TURN), math.degrees(gyaw)))
        print('leash       %.0f s' % TIMEOUT)

        # The server's reply to a freshly started client can be lost when the
        # client's response reader is not yet discovered (2026-10-01, load ~9:
        # bt_navigator "Failed to send goal response (timeout)"). Spinning a
        # little after wait_for_server gives discovery time to finish. It
        # lowers the odds; the cancel-all in `finally` is what makes it safe.
        spin_for(n, 3)
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
            c = n.nearest() if TURN is not None else n.ahead('fused')
            if c is not None and c < (TURN_ABORT if TURN is not None else 0.30):
                print('GUARD  only %.2f m clear, cancelling' % c)
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
        fmt = lambda v: '%.2f' % v if v is not None else 'none'
        print('clearance   fused %s  raw %s (was %s / %s)'
              % (fmt(n.ahead('fused')), fmt(n.ahead('raw')), fmt(cf), fmt(cr)))

        status = res_fut.result().status if res_fut.done() else -1
        imu1 = n.imu_yaw
        imu_turn = (wrap(imu1 - imu0) if (imu0 is not None and imu1 is not None)
                    else float('nan'))
        tf_turn = (wrap(end_pose[2] - start[2]) if end_pose else float('nan'))
        if TURN is not None:
            print('turned      IMU %+.1f deg, TF %+.1f deg  (asked %+.0f)'
                  % (math.degrees(imu_turn), math.degrees(tf_turn),
                     math.degrees(TURN)))
        # One line for a harness to grep. Everything above is for a person.
        print('RESULT mode=%s status=%d elapsed=%.1f travelled=%.3f '
              'imu_turn_deg=%.1f tf_turn_deg=%.1f'
              % ('turn' if TURN is not None else 'drive', status, elapsed,
                 travelled, math.degrees(imu_turn), math.degrees(tf_turn)))

        # --- guard 2: the false-success signature ---
        if TURN is not None:
            if status == 4 and not (abs(imu_turn) >= 0.3 * abs(TURN)):
                print('\n*** FALSE SUCCESS: Nav2 reported the turn done after '
                      '%.1f s; the IMU says %.1f deg. Frozen-TF signature. ***'
                      % (elapsed, math.degrees(imu_turn)))
                return 2
            return 0
        med1 = n.ahead_median()
        lidar_prog = (med0 - med1) if (med0 is not None and med1 is not None) else None
        print('lidar ahead median %s -> %s  (progress %s)'
              % (fmt(med0), fmt(med1), fmt(lidar_prog)))
        if status == 4 and end_pose:
            verdict = drive_verdict(start, end_pose, (gx, gy), XY_TOL,
                                    lidar_progress=lidar_prog)
            left = math.hypot(gx - end_pose[0], gy - end_pose[1])
            print('verdict     %s (map pose %.3f m from goal, tolerance %.2f)'
                  % (verdict, left, XY_TOL))
            if verdict == 'pose_disagrees':
                print('NOTE  the lidar says the robot arrived; Cartographer\'s map '
                      'pose says it is short. Not a TF freeze.')
            if verdict == 'false_success':
                print('\n*** FALSE SUCCESS: Nav2 reported the goal reached after '
                      '%.1f s; map pose and lidar both put the robot short '
                      '(%.3f m from it, lidar progress %s). Check controller_server '
                      'for "Transform data too old": a frozen Transform time is '
                      'the TF freeze. ***' % (elapsed, left, fmt(lidar_prog)))
                return 2
        return 0
    finally:
        # --- guard 3: stop on every exit path ---
        try:
            if handle is not None:
                handle.cancel_goal_async()
                spin_for(n, 2)
            elif sent is not None:
                # A goal went out but no handle came back. Nav2 may still
                # have accepted it and be driving, with nothing here able to
                # cancel it by ID, so cancel EVERY goal on the server: a zero
                # goal ID and zero stamp mean "all" in CancelGoal.
                print('cancel-all  no goal handle; cancelling every goal on '
                      'navigate_to_pose')
                cli = n.create_client(CancelGoal,
                                      'navigate_to_pose/_action/cancel_goal')
                if cli.wait_for_service(timeout_sec=5.0):
                    cli.call_async(CancelGoal.Request())
                    spin_for(n, 3)
                else:
                    print('cancel-all  service not found; cancel by hand')
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
