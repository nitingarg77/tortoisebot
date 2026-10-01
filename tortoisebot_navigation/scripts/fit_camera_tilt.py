#!/usr/bin/env python3
"""Measure the camera's tilt against the lidar, from wherever the robot stands.

    ros2 run tortoisebot_navigation fit_camera_tilt.py [--seconds 6] [--out tilt.png]

Commands no motion. It only subscribes, to /scan and the camera image.

floor_scan turns the row where the floor ends into a distance using the
camera's tilt, and the distance is very sensitive to it: at 5 deg against a
true 10 deg, everything comes out about half as far as it is (2026-10-01:
objects 1.2-2.7 m away reported at 0.68-0.91 m, which blocked every Nav2 goal
over 0.30 m). The one-junction calibration in floor_scan.py's docstring
cannot tell a floor junction from an edge higher up a wall; this fits every
lidar beam at once.

For each candidate tilt it puts every lidar range where that object's foot
would be in the image and scores the image's edges there (floor_geometry.
tilt_scores). Prints the score curve and the best tilt, and with --out writes
the frame with the lines for the configured and the best tilt drawn on it.
Look at that image: an edge standing above the floor makes a second peak at a
smaller tilt, and only the line that sits on the floor junctions is right.

Needs a view with opaque things at 1-3 m across much of the image. Glass and
open doorways are skipped (no lidar return, or one beyond --max-range).
"""

import argparse
import math
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from tortoisebot_navigation.floor_geometry import (
    normalise, row_for_range, tilt_scores,
)


def to_bgr(msg):
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img


def lidar_per_beam(scans, beams, hfov, dx, max_range):
    """Median lidar range for each camera beam, left to right; None if the
    lidar is no witness there. Converted to the distance ahead of base_link's
    origin, which is what the camera geometry measures from."""
    out = []
    for b in range(beams):
        ang = hfov / 2 - b * hfov / (beams - 1)
        v = []
        for m in scans:
            i = round((ang - m.angle_min) / m.angle_increment)
            for j in (i - 1, i, i + 1):
                if 0 <= j < len(m.ranges):
                    r = m.ranges[j]
                    if math.isfinite(r) and max(m.range_min, 0.05) < r:
                        v.append(r)
        d = statistics.median(v) + dx if len(v) >= len(scans) // 2 else None
        out.append(d if d is not None and d <= max_range else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--scan', default='/scan')
    ap.add_argument('--image', default='/camera_node/image_raw')
    ap.add_argument('--out', default=None, help='write an overlay PNG here')
    ap.add_argument('--configured', type=float, default=None,
                    help='tilt now in use, drawn for comparison (deg)')
    ap.add_argument('--max-range', type=float, default=4.0)
    # As floor_scan.py's defaults; pass the launch values if they differ.
    ap.add_argument('--height', type=float, default=0.134)
    ap.add_argument('--vfov', type=float, default=41.4)
    ap.add_argument('--hfov', type=float, default=53.5)
    ap.add_argument('--x-offset', type=float, default=0.05)
    ap.add_argument('--lidar-dx', type=float, default=-0.03275)
    ap.add_argument('--beams', type=int, default=60)
    args = ap.parse_args()

    rclpy.init()
    node = Node('fit_camera_tilt')
    scans, frames = [], []
    node.create_subscription(LaserScan, args.scan, scans.append,
                             qos_profile_sensor_data)
    node.create_subscription(Image, args.image,
                             lambda m: frames.append(to_bgr(m)),
                             qos_profile_sensor_data)
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not scans or not frames:
        print('FAIL: %d scans, %d frames in %.0f s' % (len(scans), len(frames),
                                                     args.seconds))
        return 1

    frame = frames[-1]
    h = frame.shape[0]
    vfov = math.radians(args.vfov)
    lidar = lidar_per_beam(scans, args.beams, math.radians(args.hfov),
                           args.lidar_dx, args.max_range)
    used = sum(d is not None for d in lidar)
    print('%d scans, %d frames, %d of %d beams have a lidar witness'
          % (len(scans), len(frames), used, args.beams))
    if used < args.beams // 4:
        print('FAIL: too few witnesses; face something opaque 1-3 m away')
        return 1

    gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edge = np.abs(cv2.Sobel(normalise(gray), cv2.CV_32F, 0, 1, ksize=3))
    pitches_deg = [p / 4 for p in range(-80, 81)]          # -20 to 20 deg
    scores = tilt_scores(edge, lidar, [math.radians(p) for p in pitches_deg],
                         vfov, args.height, args.x_offset)
    top = max(scores)
    for p, s in zip(pitches_deg, scores):
        print('%6.2f %6.1f %s' % (p, s, '#' * int(40 * s / top)))
    # Local maxima above half the best, largest tilt first: see tilt_scores.
    peaks = [(p, s) for i, (p, s) in enumerate(zip(pitches_deg, scores))
             if 0 < i < len(scores) - 1 and s >= scores[i - 1]
             and s >= scores[i + 1] and s > top / 2]
    for p, s in sorted(peaks, reverse=True):
        print('peak at %.2f deg (score %.0f)' % (p, s))

    if args.out:
        out = frame.copy()
        step = frame.shape[1] / args.beams
        lines = [(p, (0, 255, 0)) for p, _ in peaks]
        if args.configured is not None:
            lines.append((args.configured, (0, 255, 255)))
        for p, colour in lines:
            for b, d in enumerate(lidar):
                if d is None:
                    continue
                y = int(row_for_range(d, h, vfov, math.radians(p), args.height,
                                      args.x_offset))
                cv2.line(out, (int(b * step), y), (int((b + 1) * step), y),
                         colour, 2)
        cv2.putText(out, 'green: peaks %s  yellow: configured %s'
                    % (', '.join('%.2f' % p for p, _ in peaks), args.configured),
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imwrite(args.out, out)
        print('overlay written to', args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
