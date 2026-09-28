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
not a replacement for the lidar, and it is blind closer than about 0.45 m
with the camera tilted up as it currently is.

Calibrating pitch: park the robot facing a wall the lidar *can* see, note
the lidar range d and the junction row, then
    pitch_up_deg = degrees(atan(dv / fy) - atan(height / d))
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def range_from_row(row, image_height, vfov, pitch, height, x_offset=0.0):
    """Distance to whatever stands at image row `row`, or inf if it is at or
    above the horizon. Pure geometry, so it can be checked without a robot."""
    if row < 0:
        return float('inf')
    fy = (image_height / 2) / math.tan(vfov / 2)
    angle = math.atan((row - image_height / 2) / fy) - pitch
    if angle <= math.radians(0.5):
        return float('inf')
    return height / math.tan(angle) - x_offset


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
        self.pitch = math.radians(p('camera_pitch_up_deg', 4.3).value)
        self.vfov = math.radians(p('camera_vfov_deg', 41.4).value)
        self.hfov = math.radians(p('camera_hfov_deg', 53.5).value)
        self.x_offset = p('camera_x_offset', 0.05).value       # ahead of base_link
        self.width = p('process_width', 320).value
        self.beams = p('beams', 60).value
        self.min_range = p('min_range', 0.30).value
        self.max_range = p('max_range', 3.0).value
        # A junction shows up as a strong horizontal gradient. Lower finds
        # fainter edges and more false obstacles.
        self.edge_threshold = p('edge_threshold', 18.0).value
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

    # --- geometry ---------------------------------------------------------

    def _rows_to_ranges(self, rows, h):
        out = []
        for row in rows:
            d = range_from_row(row, h, self.vfov, self.pitch, self.height,
                               self.x_offset)
            out.append(d if self.min_range <= d <= self.max_range else float('inf'))
        return out

    # --- detection --------------------------------------------------------

    def _boundary_rows(self, frame):
        """For each of `beams` columns, the row where the floor ends (-1 = none)."""
        h0, w0 = frame.shape[:2]
        scale = self.width / w0
        img = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
        gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        # Horizontal edges: a floor/wall junction is a step in brightness down
        # the column, whatever the colours are.
        edge = np.abs(cv2.Sobel(gray.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3))

        # Only look below the horizon; above it there is no floor to see.
        fy = (h / 2) / math.tan(self.vfov / 2)
        horizon = int(h / 2 + fy * math.tan(self.pitch))
        rows = []
        step = max(w // self.beams, 1)
        for i in range(self.beams):
            x0 = min(i * step, w - 1)
            col = edge[:, x0:min(x0 + step, w)].max(axis=1)
            row = -1
            run = 0
            for y in range(h - 2, horizon, -1):
                run = run + 1 if col[y] > self.edge_threshold else 0
                if run >= self.run_length:
                    row = y + run - 1
                    break
            rows.append(row / scale if row >= 0 else -1)
        return rows, h0

    def _tick(self):
        frame = self.latest
        if frame is None:
            return
        rows, h = self._boundary_rows(frame)
        ranges = self._rows_to_ranges(rows, h)

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
