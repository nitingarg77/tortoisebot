#!/usr/bin/env python3
"""Stop autonomous driving when the SLAM pose estimate runs away.

The TortoiseBot has no wheel encoders, so Cartographer's odom->base_link is
lidar (and IMU) only. When scan matching falls behind, the estimate can run
away at constant or growing speed -- -559 m in one 2026-09-25 test -- and Nav2
then drives the real robot toward goals computed from a pose that is wrong.

The wheels physically cap the robot at 0.20 m/s (65 mm wheels, 60 rpm, see
differential.py), so an estimated speed well above that can only be an
estimation fault. On the first such reading this node stops the frontier
explorer, cancels every Nav2 NavigateToPose goal, and publishes zero cmd_vel,
then stays tripped until ~/reset is called.
"""


import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    from frontier_exploration_ros2.srv import ControlExploration
except ImportError:                      # explorer not built on this machine
    ControlExploration = None


from tortoisebot_navigation.runaway import RunawayDetector


class PoseWatchdog(Node):

    def __init__(self):
        super().__init__('pose_watchdog')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # 2.5x the 0.20 m/s the wheels can manage. The glitch that recovered on
        # its own in the hand-driven test peaked at 0.94 m/s and spent 2 s
        # above 0.5 m/s, so this would have caught it.
        self.max_speed = float(self.declare_parameter('max_speed', 0.5).value)
        # Speed is averaged over this window rather than taken between
        # consecutive TF samples. A runaway is sustained speed; a one-off scan
        # matching correction is not -- Cartographer snapped the pose 0.33 m
        # while the robot turned in place (2026-09-25), which a 0.5 s window
        # read as 0.63 m/s. Over 2 s that correction averages 0.17 m/s, while
        # the shortest runaway seen (~2 m in 2 s) still averages ~1 m/s.
        self.window = float(self.declare_parameter('window_s', 2.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tripped_pub = self.create_publisher(Bool, '~/tripped', 1)
        self.nav_cancel = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.explore_ctl = (self.create_client(ControlExploration, 'control_exploration')
                            if ControlExploration else None)
        self.create_service(Trigger, '~/reset', self.on_reset)

        # The trip logic lives in tortoisebot_navigation.runaway, which needs
        # no ROS and is covered by test/test_runaway.py. This is the only thing
        # that stops a robot whose localisation has diverged, and it had no
        # test at all while it lived here.
        self.detector = RunawayDetector(self.max_speed, self.window)
        self.stop_ticks = 0
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'watching {self.odom_frame}->{self.base_frame}, '
            f'tripping above {self.max_speed:.2f} m/s over {self.window:.1f} s')

    @property
    def tripped(self):
        return self.detector.tripped

    def tick(self):
        if self.tripped:
            # Keep the motors stopped for a couple of seconds after tripping,
            # in case a controller was mid-cycle when the goal was cancelled.
            if self.stop_ticks > 0:
                self.cmd_pub.publish(Twist())
                self.stop_ticks -= 1
            self.tripped_pub.publish(Bool(data=True))
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except TransformException:
            return
        t = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        p = tf.transform.translation
        if self.detector.update(t, p.x, p.y):
            self.trip(self.detector.trip_speed, p.x, p.y)
            return
        self.tripped_pub.publish(Bool(data=False))

    def trip(self, speed, x, y):
        self.stop_ticks = 20
        self.get_logger().error(
            f'pose estimate moving at {speed:.2f} m/s (limit {self.max_speed:.2f}) '
            f'at ({x:.2f}, {y:.2f}): stopping explorer, cancelling Nav2 goals. '
            f'Call ~/reset once localisation is sound again.')
        self.cmd_pub.publish(Twist())
        if self.explore_ctl is not None and self.explore_ctl.service_is_ready():
            req = ControlExploration.Request()
            req.action = ControlExploration.Request.ACTION_STOP
            self.explore_ctl.call_async(req)
        if self.nav_cancel.service_is_ready():
            # A zero goal id and zero stamp means "cancel every goal".
            self.nav_cancel.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn('Nav2 cancel service not available')

    def on_reset(self, request, response):
        self.detector.reset()
        self.get_logger().info('reset: watching again')
        response.success = True
        response.message = 'watchdog re-armed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # If this node dies while tripped, nothing else is holding the robot:
        # tripping is what cancelled the Nav2 goals, and the only thing keeping
        # the wheels still is this node republishing zero. Leave one last stop
        # behind on the way out rather than releasing a robot that was stopped
        # for a reason. Harmless when it was never tripped, because zero is
        # what a stopped robot is already being sent.
        try:
            if node.tripped:
                for _ in range(5):
                    node.cmd_pub.publish(Twist())
                    time.sleep(0.02)
        except Exception:
            pass          # shutting down anyway; differential.py times out in 1 s
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
