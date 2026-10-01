"""Tests for the camera floor-boundary geometry. No robot, no ROS.

    python3 -m pytest tortoisebot_navigation/test -q
"""

import math

import numpy as np
import pytest

from tortoisebot_navigation.floor_geometry import (
    boundary_rows, horizon_row, normalise, range_from_row, row_for_range,
    rows_to_ranges, tilt_scores,
)

# As mounted on this robot (CLAUDE.md section 7): measured, not nominal.
HEIGHT = 0.134
PITCH = math.radians(10.1)     # tilted up, which is why it is blind close in
VFOV = math.radians(41.4)
X_OFFSET = 0.05
IMAGE_H = 480


def loop_reference(edge, beams, threshold, run_length, horizon):
    """The Python loop the numpy version replaced in 176a4d1.

    Kept so the rewrite stays pinned. Scanning upward from the bottom, the
    first run of `run_length` strong rows wins, and the row reported is the
    bottom of that run.
    """
    h, w = edge.shape[:2]
    step = max(w // beams, 1)
    rows = []
    for i in range(beams):
        x0 = min(i * step, w - 1)
        col = edge[:, x0:min(x0 + step, w)].max(axis=1)
        row, run = -1, 0
        for y in range(h - 2, horizon, -1):
            run = run + 1 if col[y] > threshold else 0
            if run >= run_length:
                row = y + run - 1
                break
        rows.append(float(row))
    return np.array(rows, dtype=np.float32)


class TestRangeFromRow:

    def test_no_boundary_is_infinitely_far(self):
        assert range_from_row(-1, IMAGE_H, VFOV, PITCH, HEIGHT) == float('inf')

    def test_above_the_horizon_is_infinitely_far(self):
        # There is no floor up there to be a distance to.
        assert range_from_row(0, IMAGE_H, VFOV, PITCH, HEIGHT) == float('inf')

    def test_lower_in_the_image_is_nearer(self):
        near = range_from_row(470, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        far = range_from_row(400, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        assert near < far

    def test_the_blind_zone_this_mounting_creates(self):
        # Tilted 10.1 degrees up, the bottom of the frame lands about 0.67 m
        # out, so the camera cannot see a door frame it is about to touch.
        # If the bracket is re-aimed this moves.
        closest = range_from_row(IMAGE_H, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        assert 0.62 < closest < 0.72

    def test_x_offset_shifts_the_measurement_to_base_link(self):
        with_offset = range_from_row(470, IMAGE_H, VFOV, PITCH, HEIGHT, 0.05)
        without = range_from_row(470, IMAGE_H, VFOV, PITCH, HEIGHT, 0.0)
        assert without - with_offset == pytest.approx(0.05)

    # Measured 2026-10-01: floor_scan's junction rows (of 480) on an opaque
    # wooden panel, with the lidar's range at the same bearing, moved to
    # base_link. The panel is the left edge of the view, +19.5 to +26.8 deg.
    PANEL = [(402, 2.15), (400, 2.23), (398, 2.36), (396, 2.36), (394, 2.51),
             (394, 2.63), (392, 2.63), (392, 2.65), (390, 2.65)]

    def test_agrees_with_the_lidar_measurement_taken_on_the_robot(self):
        # At the measured tilt the camera reads the panel 0-20% short: a few
        # rows low, on the dark line where the panel meets the glossy floor.
        for row, lidar in self.PANEL:
            cam = range_from_row(row, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
            assert 0.80 * lidar < cam <= lidar, (row, lidar, cam)

    def test_the_old_tilt_halved_every_range(self):
        # The 5.0 deg in use until 2026-10-01 put the same panel at 0.78-0.87 m
        # instead of 2.2-2.7 m: the "0.68 m band" that blocked Nav2 goals.
        old = math.radians(5.0)
        for row, lidar in self.PANEL:
            cam = range_from_row(row, IMAGE_H, VFOV, old, HEIGHT, X_OFFSET)
            assert cam < 0.40 * lidar, (row, lidar, cam)

    def test_pitching_the_camera_down_shrinks_the_blind_zone(self):
        # The proposed bracket change: aim it down instead of up.
        up = range_from_row(IMAGE_H, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        down = range_from_row(IMAGE_H, IMAGE_H, VFOV, math.radians(-5.0),
                              HEIGHT, X_OFFSET)
        assert down < up


class TestRowForRange:

    @pytest.mark.parametrize('d', [0.7, 1.0, 1.5, 2.5])
    def test_inverts_range_from_row(self, d):
        row = row_for_range(d, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        assert range_from_row(row, IMAGE_H, VFOV, PITCH, HEIGHT,
                              X_OFFSET) == pytest.approx(d)


def synthetic_edges(lidar, pitch, height=HEIGHT, width=640):
    """A vertical-gradient image with one strong edge per beam, at the row
    where an object `lidar[b]` away meets the floor -- or, with `height`
    below the camera's, where an edge that high up the object appears."""
    edge = np.zeros((IMAGE_H, width), np.float32)
    step = width // len(lidar)
    for b, d in enumerate(lidar):
        if d is None:
            continue
        row = int(round(row_for_range(d, IMAGE_H, VFOV, pitch, height,
                                      X_OFFSET)))
        edge[row - 1:row + 2, b * step:(b + 1) * step] = 100.0
    return edge


class TestTiltScores:

    PITCHES = [math.radians(p / 4) for p in range(0, 61)]     # 0-15 deg

    def best(self, scores):
        # The middle of the top: a +-3 row band is about +-0.3 deg, so
        # neighbouring tilts tie (on the robot: 10.0-10.25 deg).
        top = [i for i, s in enumerate(scores) if s >= max(scores) - 1e-6]
        return math.degrees(self.PITCHES[(top[0] + top[-1]) // 2])

    def test_finds_the_tilt_the_edges_were_drawn_with(self):
        lidar = [1.0 + 0.03 * b for b in range(60)]
        for true in (4.0, 10.0):
            edge = synthetic_edges(lidar, math.radians(true))
            scores = tilt_scores(edge, lidar, self.PITCHES, VFOV, HEIGHT,
                                 X_OFFSET)
            assert self.best(scores) == pytest.approx(true, abs=0.25)

    def test_beams_without_a_witness_are_skipped(self):
        # Glass: the lidar sees through it, so those beams must not vote.
        lidar = [None if b % 3 else 1.8 for b in range(60)]
        edge = synthetic_edges(lidar, math.radians(10.0))
        scores = tilt_scores(edge, lidar, self.PITCHES, VFOV, HEIGHT, X_OFFSET)
        assert self.best(scores) == pytest.approx(10.0, abs=0.25)
        assert tilt_scores(edge, [None] * 60, self.PITCHES, VFOV, HEIGHT,
                           X_OFFSET) == [0.0] * len(self.PITCHES)

    def test_a_raised_edge_peaks_at_a_smaller_tilt(self):
        # What happened on 2026-10-01: a plinth's top edge lined up with the
        # lidar too, at 6.6 deg against the floor's 10.1. The floor is the
        # larger-tilt peak, which is why the tool says to check by eye.
        lidar = [2.0 + 0.01 * b for b in range(60)]
        edge = np.maximum(synthetic_edges(lidar, math.radians(10.0)),
                          synthetic_edges(lidar, math.radians(10.0),
                                          height=HEIGHT - 0.12))
        scores = tilt_scores(edge, lidar, self.PITCHES, VFOV, HEIGHT, X_OFFSET)
        peaks = [math.degrees(self.PITCHES[i]) for i in range(1, len(scores) - 1)
                 if scores[i] >= scores[i - 1] and scores[i] >= scores[i + 1]
                 and scores[i] > max(scores) / 2]
        assert len(peaks) >= 2
        assert max(peaks) == pytest.approx(10.0, abs=0.25)


class TestHorizonRow:

    def test_tilted_up_puts_the_horizon_below_centre(self):
        assert horizon_row(IMAGE_H, VFOV, PITCH) > IMAGE_H / 2

    def test_level_puts_it_at_centre(self):
        assert horizon_row(IMAGE_H, VFOV, 0.0) == IMAGE_H // 2


class TestNormalise:

    def test_a_dim_frame_and_a_bright_one_land_together(self):
        rng = np.random.default_rng(0)
        base = rng.integers(40, 80, (48, 64)).astype(np.uint8)
        assert np.median(normalise(base)) == pytest.approx(128, abs=2)
        assert np.median(normalise((base * 2).astype(np.uint8))) == \
            pytest.approx(128, abs=2)

    def test_a_black_frame_does_not_divide_by_zero(self):
        # The camera really does return an all-black frame when auto exposure
        # is disabled at startup, so this is a case that has happened.
        assert np.isfinite(normalise(np.zeros((16, 16), np.uint8))).all()


class TestBoundaryRows:

    @pytest.mark.parametrize('seed', range(40))
    def test_matches_the_loop_it_replaced(self, seed):
        rng = np.random.default_rng(seed)
        run_length = int(rng.integers(1, 6))
        horizon = int(rng.integers(0, 200))
        edge = (rng.random((240, 320)).astype(np.float32)
                * float(rng.choice([60, 90, 200])))
        assert np.array_equal(
            boundary_rows(edge, 60, 45.0, run_length, horizon),
            loop_reference(edge, 60, 45.0, run_length, horizon))

    def test_finds_a_clean_horizontal_junction(self):
        edge = np.zeros((240, 320), np.float32)
        edge[150:153, :] = 200.0
        rows = boundary_rows(edge, 60, 45.0, 3, 100)
        assert np.all(rows == 152)

    def test_reports_the_nearest_junction_when_there_are_two(self):
        # Lower in the image is nearer, and the near one is what can be hit.
        edge = np.zeros((240, 320), np.float32)
        edge[120:123, :] = 200.0
        edge[180:183, :] = 200.0
        assert np.all(boundary_rows(edge, 60, 45.0, 3, 100) == 182)

    def test_an_empty_frame_finds_nothing(self):
        rows = boundary_rows(np.zeros((240, 320), np.float32), 60, 45.0, 3, 100)
        assert np.all(rows == -1)

    def test_ignores_edges_above_the_horizon(self):
        edge = np.zeros((240, 320), np.float32)
        edge[50:53, :] = 200.0
        assert np.all(boundary_rows(edge, 60, 45.0, 3, 150) == -1)

    def test_a_streak_shorter_than_run_length_is_noise(self):
        edge = np.zeros((240, 320), np.float32)
        edge[150:152, :] = 200.0        # 2 rows, run_length 3
        assert np.all(boundary_rows(edge, 60, 45.0, 3, 100) == -1)

    def test_neighbour_filter_drops_an_isolated_column(self):
        # The marble veining case: one column disagreeing wildly with its
        # neighbours is not a floor boundary, it is a pattern in the floor.
        edge = np.zeros((240, 320), np.float32)
        edge[150:153, :] = 200.0
        edge[150:153, 40:45] = 0.0
        edge[200:203, 40:45] = 200.0
        rows = boundary_rows(edge, 60, 45.0, 3, 100, neighbour_tolerance=12)
        assert (rows == -1).any()
        assert (rows == 152).sum() > 50

    def test_horizon_past_the_bottom_of_the_frame_is_handled(self):
        rows = boundary_rows(np.zeros((240, 320), np.float32), 60, 45.0, 3, 239)
        assert np.all(rows == -1)

    def test_beams_that_do_not_divide_the_width(self):
        edge = np.zeros((240, 322), np.float32)
        edge[150:153, :] = 200.0
        assert len(boundary_rows(edge, 60, 45.0, 3, 100)) == 60


class TestRowsToRanges:

    def test_out_of_range_becomes_inf_not_a_wrong_number(self):
        # A range outside the sensor's declared limits must be dropped, not
        # clipped: a clipped value is an obstacle the costmap will mark.
        rows = [-1, IMAGE_H - 1, 300]
        out = rows_to_ranges(rows, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET,
                             0.30, 3.0)
        assert out[0] == float('inf')
        assert all(r == float('inf') or 0.30 <= r <= 3.0 for r in out)

    def test_every_row_yields_a_float(self):
        out = rows_to_ranges(list(range(0, IMAGE_H, 40)), IMAGE_H, VFOV, PITCH,
                             HEIGHT, X_OFFSET, 0.30, 3.0)
        assert all(isinstance(r, float) for r in out)
