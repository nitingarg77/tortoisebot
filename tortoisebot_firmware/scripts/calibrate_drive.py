#!/usr/bin/env python3
"""Measure how much the robot curves when told to go straight.

    python3 calibrate.py           # 3 out-and-back runs of 2.0 m
    python3 calibrate.py 4 1.5     # 4 runs of 1.5 m

Drives out and back so it stays in one lane, and reports every run rather than
an average alone: a single run already misled me once, by coming out six times
smaller than the one before it with nothing changed.

Heading comes from the BNO055, which is a fused absolute yaw rather than an
integrated gyro, so it does not wander over the ~15 s a run takes. Distance
comes from the lidar in the direction of travel. Every leg is guarded and will
stop early rather than drive into anything.
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from rclpy.qos import qos_profile_sensor_data

SPEED = 0.15
GUARD = 0.60
LEG_TIMEOUT = 60.0


def cone(scan, centre_deg, half=8):
    if scan is None:
        return None
    rs = []
    for i, r in enumerate(scan.ranges):
        if not (0 < r < 50) or r != r:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        if abs((a - centre_deg + 180) % 360 - 180) <= half:
            rs.append(r)
    return sorted(rs)[len(rs) // 2] if rs else None


class Rig:
    def __init__(self):
        self.n = rclpy.create_node('calibrate')
        self.pub = self.n.create_publisher(Twist, 'cmd_vel', 10)
        self.st = {}
        self.n.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data)
        self.n.create_subscription(LaserScan, '/scan',
                                   lambda m: self.st.update(s=m), qos_profile_sensor_data)
        t = time.time()
        while time.time() - t < 10 and ('yaw' not in self.st or 's' not in self.st):
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if 'yaw' not in self.st or 's' not in self.st:
            sys.exit('need both /imu and /scan')

    def _imu(self, m):
        q = m.orientation
        if abs(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) - 1.0) < 0.05:
            self.st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y**2 + q.z**2))

    def leg(self, forward, want):
        face = 0 if forward else 180
        d0 = cone(self.st['s'], face)
        if d0 is None or d0 - want < GUARD:
            return None, None, 'only %s m that way' % (
                '%.2f' % d0 if d0 is not None else '?')
        cmd = Twist()
        last, turned, gone = self.st['yaw'], 0.0, 0.0
        began, why = time.time(), 'timeout'
        try:
            while time.time() - began < LEG_TIMEOUT:
                rclpy.spin_once(self.n, timeout_sec=0.02)
                y = self.st['yaw']
                turned += math.atan2(math.sin(y - last), math.cos(y - last))
                last = y
                d = cone(self.st['s'], face)
                if d is not None:
                    gone = d0 - d
                    if d <= GUARD:
                        why = 'guard at %.2f m' % d
                        break
                    if gone >= want:
                        why = 'ok'
                        break
                cmd.linear.x = SPEED if forward else -SPEED
                self.pub.publish(cmd)
                time.sleep(0.02)
        finally:
            for _ in range(10):
                try:
                    self.pub.publish(Twist())
                except Exception:
                    break
                time.sleep(0.03)
        for _ in range(12):
            rclpy.spin_once(self.n, timeout_sec=0.05)
        per_m = math.degrees(turned) / gone if gone > 0.3 else None
        return per_m, gone, why


def main():
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    dist = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    rig = Rig()
    fwd, back = [], []
    for k in range(runs):
        for forward, bucket, name in ((True, fwd, 'fwd '), (False, back, 'back')):
            per_m, gone, why = rig.leg(forward, dist)
            if per_m is None:
                print('run %d %s: SKIPPED (%s)' % (k + 1, name, why), flush=True)
                continue
            bucket.append(per_m)
            print('run %d %s: %+6.1f deg/m over %.2f m   (%s)'
                  % (k + 1, name, per_m, gone, why), flush=True)
            time.sleep(1.0)
    print()
    for name, vals in (('forward', fwd), ('reverse', back)):
        if not vals:
            print('%s: no valid runs' % name)
            continue
        m = statistics.mean(vals)
        sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
        print('%s: mean %+.1f deg/m, spread +/-%.1f, n=%d  %s'
              % (name, m, sd, len(vals), ['%+.1f' % v for v in vals]))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
