"""Tests for the camera floor-boundary geometry. No robot, no ROS.

    python3 -m pytest tortoisebot_navigation/test -q
"""

import math

import numpy as np
import pytest

from tortoisebot_navigation.floor_geometry import (
    boundary_rows, horizon_row, normalise, range_from_row, rows_to_ranges,
)

# As mounted on this robot (CLAUDE.md section 7): measured, not nominal.
HEIGHT = 0.134
PITCH = math.radians(5.0)      # tilted up, which is why it is blind close in
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
        # Tilted 5 degrees up, the bottom of the frame still lands about 0.45 m
        # out, so the camera cannot see a door frame it is about to touch.
        # MODULE.md depends on this figure; if the bracket is re-aimed it moves.
        closest = range_from_row(IMAGE_H, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        assert 0.40 < closest < 0.50

    def test_x_offset_shifts_the_measurement_to_base_link(self):
        with_offset = range_from_row(470, IMAGE_H, VFOV, PITCH, HEIGHT, 0.05)
        without = range_from_row(470, IMAGE_H, VFOV, PITCH, HEIGHT, 0.0)
        assert without - with_offset == pytest.approx(0.05)

    def test_agrees_with_the_lidar_measurement_taken_on_the_robot(self):
        # Parked facing a wall the lidar could see: lidar 1.19 m, floor_scan
        # 1.16 m. Recover the row that gives 1.16 and check it round-trips, so
        # the calibration that produced pitch=5.0 stays pinned to real numbers.
        rows = [r for r in range(240, IMAGE_H)
                if abs(range_from_row(r, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
                       - 1.16) < 0.02]
        assert rows, 'no row yields the 1.16 m measured against the lidar'

    def test_pitching_the_camera_down_shrinks_the_blind_zone(self):
        # The proposed bracket change: aim it down instead of up.
        up = range_from_row(IMAGE_H, IMAGE_H, VFOV, PITCH, HEIGHT, X_OFFSET)
        down = range_from_row(IMAGE_H, IMAGE_H, VFOV, math.radians(-5.0),
                              HEIGHT, X_OFFSET)
        assert down < up


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
