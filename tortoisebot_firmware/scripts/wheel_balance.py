"""Drive one wheel at a time and see whether the two match.

With the other wheel held at zero the robot pivots about it, so the yaw rate
the IMU reports is a direct measure of that one wheel's speed. A healthy pair
gives equal and opposite rates. A weak or intermittent motor shows up as a
smaller rate, or as a rate that changes between repeats of the same command.

cmd_vel is chosen so one wheel gets exactly zero:
    left = 0  ->  v = R/2, w = +R/L
    right = 0 ->  v = L/2, w = -L/L
which only works now that a wheel asked for zero is given zero duty; it used
to be floored at 15%.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from rclpy.qos import qos_profile_sensor_data

# Shared with the other drive tools and covered by
# tortoisebot_navigation/test/test_scan_geometry.py. cone() in
# particular: used as an odometer through a doorway it reported
# negative progress and hid a 6.8 m overshoot.
from tortoisebot_navigation.scan_geometry import nearest as _nearest


def nearest(scan):
    """LaserScan wrapper. Returns inf for a missing or empty scan,
    so a guard cannot read 'saw nothing' as 'touching something'."""
    return float('inf') if scan is None else _nearest(scan.ranges)

L = 0.17
WHEEL = 0.15          # m/s asked of the moving wheel
BURST = 2.0
GUARD = 0.35


rclpy.init()
n = rclpy.create_node('wheeltest')
pub = n.create_publisher(Twist, 'cmd_vel', 10)
st = {}


def on_imu(m):
    q = m.orientation
    if abs(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) - 1.0) < 0.05:
        st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y**2 + q.z**2))


n.create_subscription(Imu, '/imu', on_imu, qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan', lambda m: st.update(s=m),
                      qos_profile_sensor_data)
t = time.time()
while time.time() - t < 10 and ('yaw' not in st or 's' not in st):
    rclpy.spin_once(n, timeout_sec=0.1)

print('%-16s %-12s %s' % ('driving', 'yaw rate', 'implied wheel speed'))
for rep in range(3):
    for name, v, w in (('RIGHT wheel', WHEEL / 2, WHEEL / L),
                       ('LEFT wheel', WHEEL / 2, -WHEEL / L)):
        if nearest(st.get('s')) < GUARD:
            print('  too close to anything (%.2f m); stopping'
                  % nearest(st.get('s')))
            break
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        last, turned = st['yaw'], 0.0
        end = time.time() + BURST
        # The stop goes in a finally: an exception raised mid-burst would
        # otherwise leave the robot driving until differential.py's 1 s cmd_vel
        # timeout notices, and a tool that commands motion must stop it on
        # every exit path (CLAUDE.md section 2).
        try:
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.02)
                y = st['yaw']
                turned += math.atan2(math.sin(y - last), math.cos(y - last))
                last = y
                if nearest(st.get('s')) < GUARD:
                    print('  guard: something at %.2f m, stopping the burst'
                          % nearest(st.get('s')))
                    break
                pub.publish(cmd)
                time.sleep(0.02)
        finally:
            for _ in range(10):
                try:
                    pub.publish(Twist())
                except Exception:
                    break        # context already gone; the 1 s timeout covers it
                time.sleep(0.03)
        rate = turned / BURST
        print('%-16s %+7.2f rad/s  %.3f m/s  (asked %.3f)'
              % ('%s #%d' % (name, rep + 1), rate, abs(rate) * L, WHEEL),
              flush=True)
        time.sleep(1.2)
n.destroy_node()
