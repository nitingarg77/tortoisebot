"""Where the floor ends, in pixels and in metres. No ROS, so it tests at a desk.

Split out of floor_scan.py, which imports rclpy at module scope and therefore
cannot be reached without a robot. The numpy rewrite in 176a4d1 was checked
against 400 random frames -- but against a copy of the logic pasted into a
throwaway script, not against the function that ships. That proves the
algorithm and nothing about the code. This module is the code, and the tests
import it.

The camera measures no distance on its own. What it can measure is the image
row where the floor stops: with the camera at a known height and tilt, a pixel
`dv` below the optical axis looks down at `atan(dv / fy) - pitch`, and the
floor is `height / tan(that angle)` away.
"""

import math

import cv2
import numpy as np


def range_from_row(row, image_height, vfov, pitch, height, x_offset=0.0):
    """Distance to whatever stands at image row `row`, or inf at or above the
    horizon. Pure geometry, so it can be checked without a robot.

    Calibrating pitch: park facing a wall the lidar *can* see, note the lidar
    range d and the junction row, then
        pitch_up_deg = degrees(atan(dv / fy) - atan(height / d))
    """
    if row < 0:
        return float('inf')
    fy = (image_height / 2) / math.tan(vfov / 2)
    angle = math.atan((row - image_height / 2) / fy) - pitch
    if angle <= math.radians(0.5):
        return float('inf')
    return height / math.tan(angle) - x_offset


def row_for_range(distance, image_height, vfov, pitch, height, x_offset=0.0):
    """The image row where the floor meets something `distance` away.

    The inverse of range_from_row, for lining the lidar up against the image.
    """
    fy = (image_height / 2) / math.tan(vfov / 2)
    angle = math.atan(height / (distance + x_offset)) + pitch
    return image_height / 2 + fy * math.tan(angle)


def tilt_scores(edge, lidar, pitches, vfov, height, x_offset, band=3):
    """How well each candidate pitch lines the lidar up with the image's edges.

    `edge` is a vertical-gradient image; `lidar` is one range per beam (left
    to right across the image, as boundary_rows' columns are), None where the
    lidar is no witness -- no return, or a range through glass. For each
    pitch, every beam's lidar range is turned into the row where that
    object's foot should be, and the score is the mean of the strongest edge
    within `band` rows of it. A right pitch puts every beam on a real floor
    junction at once; a wrong one puts them on blank wall or blank floor.

    Expect a second, smaller-pitch peak from edges standing above the floor
    (a plinth's top, a skirting board): they line up with the lidar too, just
    higher. The floor junction is the peak with the larger pitch -- check it
    by eye with fit_camera_tilt.py's overlay before trusting either.
    """
    h, w = edge.shape[:2]
    beams = len(lidar)
    step = max(w // beams, 1)
    scores = []
    for pitch in pitches:
        total, used = 0.0, 0
        for b, d in enumerate(lidar):
            if d is None:
                continue
            row = int(round(row_for_range(d, h, vfov, pitch, height, x_offset)))
            if row - band < 0 or row + band + 1 > h:
                continue
            total += float(edge[row - band:row + band + 1,
                                b * step:(b + 1) * step].max(axis=0).mean())
            used += 1
        scores.append(total / used if used else 0.0)
    return scores


def horizon_row(image_height, vfov, pitch):
    """Rows at or above this see no floor, so there is nothing to look for."""
    fy = (image_height / 2) / math.tan(vfov / 2)
    return int(image_height / 2 + fy * math.tan(pitch))


def normalise(gray):
    """Scale to a fixed median brightness.

    The camera's auto exposure changes the picture's level whenever the view
    changes, so an absolute edge threshold would mean something different every
    frame. Disabling auto exposure at startup is not the answer: the camera
    then never exposes at all and the image comes out black.
    """
    median = float(np.median(gray))
    return np.clip(gray.astype(np.float32) * (128.0 / max(median, 1.0)), 0, 255)


def boundary_rows(edge, beams, threshold, run_length, horizon,
                  neighbour_tolerance=0):
    """For each of `beams` column blocks, the lowest row where the floor ends.

    -1 where no boundary was found. `edge` is a vertical-gradient image; a
    floor/wall junction is a step in brightness down the column, whatever the
    colours are.

    Threshold swept on the real floor: 18 found the marble's veining (61 false
    readings in 180 beams), 30 still found 10, 45 found none while keeping
    177/180 real detections, and 60 started losing the junction itself.
    """
    h, w = edge.shape[:2]
    step = max(w // beams, 1)
    usable = step * beams
    strong = edge[:, :usable].reshape(h, beams, step).max(axis=2) > threshold

    # run[y] marks a stretch of `run_length` strong rows whose bottom row is y.
    run = strong.copy()
    for k in range(1, run_length):
        run[k:] &= strong[:-k]

    # Take the lowest such stretch -- the nearest one -- and require all of it
    # to sit below the horizon.
    window = run[min(horizon + run_length, h - 1):h - 1]
    if window.shape[0] == 0:
        rows = np.full(beams, -1.0, dtype=np.float32)
    else:
        # argmax down the reversed window is the first hit from the bottom.
        rows = np.where(window.any(axis=0),
                        (h - 2) - window[::-1].argmax(axis=0),
                        -1).astype(np.float32)

    if neighbour_tolerance > 0 and (rows >= 0).sum() > 5:
        # A real floor boundary is continuous across neighbouring columns, so
        # drop any column that disagrees with its neighbours by too much.
        smooth = cv2.medianBlur(rows.reshape(1, -1), 5).ravel()
        rows[np.abs(rows - smooth) > neighbour_tolerance] = -1
    return rows


def rows_to_ranges(rows, image_height, vfov, pitch, height, x_offset,
                   min_range, max_range):
    """Boundary rows to laser ranges, with anything out of range as inf."""
    out = []
    for row in rows:
        d = range_from_row(row, image_height, vfov, pitch, height, x_offset)
        out.append(d if min_range <= d <= max_range else float('inf'))
    return out
