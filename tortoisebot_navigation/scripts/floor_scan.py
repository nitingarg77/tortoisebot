#!/usr/bin/env python3
"""See obstacles the lidar misses, by finding where the floor ends.

The YDLidar cannot see the white fluted panels in this building: measured
head-on at 1.2 m, the beams straight ahead returned nothing in 55-80% of
scans, because the ribs reflect the beam away. The robot drove into them.

A camera does see them, but a single camera measures no distance. What it can
measure is the row in the image where the floor stops. With the camera fixed
at a known height and tilt, that row is a distance: a pixel `dv` below the
optical axis looks down at `atan(dv / fy) - pitch`, and the floor is
`height / tan(that angle)` away.

The result is published as a LaserScan so Nav2 can use it as a second
obstacle source next to the real lidar. It only ever adds obstacles; it is
not a replacement for the lidar, and it is blind closer than about 0.24 m
from base_link with the camera tilted 4.5 deg down as it currently is
(0.67 m at the 10.1 deg up it was mounted at before 2026-10-01).

Calibrating pitch: run fit_camera_tilt.py facing opaque things 1-3 m away,
and check its overlay. The single-junction formula
    pitch_up_deg = degrees(atan(dv / fy) - atan(height / d))
is what produced the wrong 5.0: one junction cannot show whether the row
picked is the floor or an edge higher up.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

# The geometry and the boundary search live in a module that imports no ROS,
# so they are covered by tortoisebot_navigation/test/test_floor_geometry.py
# without a robot. This file cannot be imported without rclpy, so anything
# defined here can only be tested by driving.
from tortoisebot_navigation.floor_geometry import (
    boundary_rows, horizon_row, normalise, range_from_row, rows_to_ranges,
)


def to_bgr(msg):
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {msg.encoding!r}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


class FloorScan(Node):

    def __init__(self):
        super().__init__('floor_scan')
        p = self.declare_parameter
        self.height = p('camera_height', 0.134).value          # m above floor
        # The launch file sets the measured value; see the note there.
        self.pitch = math.radians(p('camera_pitch_up_deg', -4.5).value)
        self.vfov = math.radians(p('camera_vfov_deg', 41.4).value)
        self.hfov = math.radians(p('camera_hfov_deg', 53.5).value)
        self.x_offset = p('camera_x_offset', 0.05).value       # ahead of base_link
        self.width = p('process_width', 320).value
        self.beams = p('beams', 60).value
        self.min_range = p('min_range', 0.30).value
        self.max_range = p('max_range', 3.0).value
        # A junction shows up as a strong horizontal gradient. Swept on the
        # real floor: 18 found the marble's veining (61 false readings in 180
        # beams), 30 still found 10, 45 found none while keeping 177/180 real
        # detections, and 60 started losing the junction itself.
        self.edge_threshold = p('edge_threshold', 45.0).value
        # A real floor boundary is continuous across neighbouring columns, so
        # drop any column that disagrees with its neighbours by more than this
        # many rows (at the processing size). 0 disables the check.
        self.neighbour_tolerance = p('neighbour_tolerance_rows', 12).value
        self.run_length = p('edge_run_length', 3).value
        self.rate = p('rate_hz', 5.0).value
        self.frame_id = p('frame_id', 'base_link').value

        self.latest = None
        self.create_subscription(Image, p('image_topic', 'camera/image_raw').value,
                                 self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, 'floor_scan', 10)
        self.create_timer(1.0 / max(self.rate, 0.5), self._tick)
        self.get_logger().info(
            f'floor_scan: height {self.height:.3f} m, '
            f'pitch {math.degrees(self.pitch):+.1f} deg up, sees floor from '
            f'{range_from_row(480, 480, self.vfov, self.pitch, self.height, self.x_offset):.2f} m')

    def _on_image(self, msg):
        try:
            self.latest = to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)

    # --- detection --------------------------------------------------------

    def _boundary_rows(self, frame):
        """For each of `beams` columns, the row where the floor ends (-1 = none)."""
        h0, w0 = frame.shape[:2]
        scale = self.width / w0
        img = cv2.resize(frame, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
        gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        # Horizontal edges: a floor/wall junction is a step in brightness down
        # the column, whatever the colours are.
        edge = np.abs(cv2.Sobel(normalise(gray), cv2.CV_32F, 0, 1, ksize=3))
        rows = boundary_rows(edge, self.beams, self.edge_threshold,
                             self.run_length,
                             horizon_row(img.shape[0], self.vfov, self.pitch),
                             self.neighbour_tolerance)
        return [r / scale if r >= 0 else -1 for r in rows], h0

    def _tick(self):
        frame = self.latest
        if frame is None:
            return
        rows, h = self._boundary_rows(frame)
        ranges = rows_to_ranges(rows, h, self.vfov, self.pitch, self.height,
                                self.x_offset, self.min_range, self.max_range)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -self.hfov / 2
        msg.angle_max = self.hfov / 2
        msg.angle_increment = self.hfov / max(len(ranges) - 1, 1)
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        # Column 0 is the left of the image, which is +angle in ROS, so the
        # ranges run right to left.
        msg.ranges = [float(r) for r in reversed(ranges)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorScan()
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
