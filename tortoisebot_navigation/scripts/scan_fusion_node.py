#!/usr/bin/env python3
"""Publish /scan_fused: the lidar scan with the camera's obstacles folded in.

A thin shell. Everything that decides anything lives in
tortoisebot_navigation/scan_fusion.py, which imports no ROS and is covered by
test/test_scan_fusion.py without a robot. Read that module's docstring for the
rule and why the obvious version of it is wrong.

What this file owns is the ROS wiring and the concurrency:

- It publishes on every lidar scan, stamped with the **lidar's** stamp, never
  `now()` -- the costmap transforms at that stamp.
- It publishes even when the camera is dead, because /scan_fused is the
  costmaps' only observation source and silence blinds Nav2 completely.
- It publishes nothing when the lidar is dead. A fabricated scan would read as
  clear space.
- If fusion raises for any reason it republishes the raw lidar scan and carries
  on. This node is now the costmaps' only observation source, which the lidar
  driver used to be; a bug in it must degrade to what the lidar said, not take
  Nav2's eyes out. An uncaught exception in an rclpy callback propagates out of
  `spin`, so without this the first bad message would end the node.

Wiring (nav2_params_robot.yaml): both costmaps take this one topic instead of
`scan` plus `floor_scan`. Two observation sources meant two
`tf2_ros::MessageFilter` per layer, and MODULE.md records what that did to
`controller_server`. Cartographer is untouched: it remaps `scan` explicitly and
keeps the raw lidar.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

from tortoisebot_navigation.places import yaw_from_quaternion
from tortoisebot_navigation.scan_fusion import (
    DEFAULT_CLUTTER_FRACTION, DEFAULT_CLUTTER_RANGE, DEFAULT_MAX_AGE,
    DEFAULT_UNKNOWN_YAW_MAX_AGE, LIDAR_DX, LIDAR_DY, Fan, YawHistory, fuse,
)


class ScanFusion(Node):

    def __init__(self):
        super().__init__('scan_fusion')
        p = self.declare_parameter
        self.max_age = p('max_camera_age', DEFAULT_MAX_AGE).value
        self.unknown_yaw_max_age = p('unknown_yaw_max_age',
                                     DEFAULT_UNKNOWN_YAW_MAX_AGE).value
        self.clutter_fraction = p('clutter_fraction',
                                  DEFAULT_CLUTTER_FRACTION).value
        self.clutter_range = p('clutter_range', DEFAULT_CLUTTER_RANGE).value
        # The lidar's position in base_link. Defaults come from the lidar_joint
        # in tortoisebotreal.xacro; parameters so a re-mount is a config change
        # rather than an edit here.
        self.dx = p('lidar_dx', LIDAR_DX).value
        self.dy = p('lidar_dy', LIDAR_DY).value
        self.use_imu = p('use_imu', True).value

        # Guards `camera` and `yaw`. A single-threaded executor already
        # serialises these callbacks, but that is a property of how the node is
        # spun, not of this code, and CLAUDE.md section 4 says to assume
        # callbacks are starved. The lock costs nothing at 12 Hz.
        self.lock = threading.Lock()
        self.camera = None
        self.yaw = YawHistory()

        self.pub = self.create_publisher(
            LaserScan, p('output_topic', 'scan_fused').value,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, p('scan_topic', 'scan').value,
            self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, p('floor_scan_topic', 'floor_scan').value,
            self._on_camera, qos_profile_sensor_data)
        if self.use_imu:
            self.create_subscription(
                Imu, p('imu_topic', 'imu').value,
                self._on_imu, qos_profile_sensor_data)

        self.published = 0
        self.with_camera = 0
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            'scan_fusion: lidar offset (%.5f, %.5f) m, camera dropped after '
            '%.2f s, %s' % (self.dx, self.dy, self.max_age,
                            'yaw from IMU' if self.use_imu else 'no yaw '
                            'correction (use_imu false)'))

    # --- inputs: store only, decide nothing ------------------------------

    def _on_camera(self, msg):
        try:
            fan = Fan.from_msg(msg)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error('bad floor_scan message: %s' % e,
                                    throttle_duration_sec=10.0)
            return
        with self.lock:
            self.camera = fan

    def _on_imu(self, msg):
        try:
            q = msg.orientation
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error('bad imu message: %s' % e,
                                    throttle_duration_sec=10.0)
            return
        with self.lock:
            self.yaw.add(stamp, yaw)

    # --- output ----------------------------------------------------------

    def _on_lidar(self, msg):
        try:
            ranges, note = self._fuse(msg)
        except Exception as e:                       # noqa: BLE001
            # Fall through to the raw scan. Losing the camera is a degradation;
            # losing the scan is Nav2 driving blind.
            self.get_logger().error('fusion failed, passing the lidar through: '
                                    '%s' % e, throttle_duration_sec=10.0)
            ranges, note = msg.ranges, 'fusion raised'

        # Republish the lidar's own message with the ranges replaced, so every
        # other field -- stamp, frame, increments, limits -- is by construction
        # whatever the driver said rather than a copy that can drift from it.
        # rclpy deserialises a fresh message per subscription, so mutating it
        # here cannot be seen by anyone else.
        msg.ranges = [float(r) for r in ranges]
        msg.intensities = []
        self.pub.publish(msg)

        self.published += 1
        if note is None:
            self.with_camera += 1
        else:
            self.get_logger().warn('lidar only: %s' % note,
                                   throttle_duration_sec=10.0)

    def _fuse(self, msg):
        lidar = Fan.from_msg(msg)
        with self.lock:
            camera = self.camera
            # None means "nobody knows whether it turned", which fuse() treats
            # differently from zero. Without the IMU it is always unknown, and
            # the fan is then used only while it is very fresh.
            delta = (self.yaw.delta(camera.stamp, lidar.stamp)
                     if camera is not None and self.use_imu else None)

        return fuse(
            lidar, camera, yaw_delta=delta, max_age=self.max_age,
            unknown_yaw_max_age=self.unknown_yaw_max_age,
            dx=self.dx, dy=self.dy,
            clutter_fraction=self.clutter_fraction,
            clutter_range=self.clutter_range)

    def _report(self):
        """Say how much of the time the camera is actually contributing.

        Without this the failure is silent: the node keeps publishing a
        perfectly good lidar-only scan, and nothing indicates that the camera
        has stopped helping.
        """
        if self.published == 0:
            self.get_logger().error('no lidar scans received')
            return
        self.get_logger().info(
            'scan_fusion: %d scans, camera used in %.0f%%'
            % (self.published, 100.0 * self.with_camera / self.published))
        self.published = self.with_camera = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
