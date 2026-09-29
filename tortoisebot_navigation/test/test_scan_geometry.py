"""Tests for reading a LaserScan by bearing. No robot, no ROS.

Several of these encode the 6.8 m overshoot: a cone used as an odometer while
the robot curved, reporting negative progress, with the tool built on it
concluding the motors were dead.
"""

import math

import pytest

from tortoisebot_navigation.scan_geometry import (
    INF, cone, index_for, is_usable_as_odometer, nearest, travelled,
)

# The YDLidar on this robot: 260 beams over a full circle.
COUNT = 260
ANGLE_MIN = -math.pi
INCREMENT = 2 * math.pi / COUNT


def scan(default=5.0, **at_deg):
    """A full scan at `default`, overridden at the given bearings."""
    ranges = [default] * COUNT
    for deg, value in at_deg.items():
        d = float(deg.replace('n', '-').replace('d', ''))
        i = index_for(math.radians(d), ANGLE_MIN, INCREMENT, COUNT)
        for j in range(i - 3, i + 4):
            ranges[j % COUNT] = value
    return ranges


class TestCone:

    def test_reads_the_bearing_it_is_asked_for(self):
        assert cone(scan(5.0, d0=1.5), ANGLE_MIN, INCREMENT, 0) == \
            pytest.approx(1.5)

    def test_an_empty_scan_gives_none_not_zero(self):
        # None, because zero would read as 'something is touching me' and a
        # guard would fire forever.
        assert cone([INF] * COUNT, ANGLE_MIN, INCREMENT, 0) is None

    def test_nan_beams_are_ignored(self):
        ranges = scan(5.0, d0=1.5)
        for i in range(COUNT // 2 - 2, COUNT // 2 + 2):
            ranges[i] = float('nan')
        got = cone(ranges, ANGLE_MIN, INCREMENT, 0)
        assert got is None or got == got        # never NaN

    def test_median_resists_a_single_stray_beam(self):
        ranges = [5.0] * COUNT
        ranges[COUNT // 2] = 0.2                # one bad beam
        assert cone(ranges, ANGLE_MIN, INCREMENT, 0, reducer='median') == \
            pytest.approx(5.0)

    def test_min_does_not_resist_it_and_should_not(self):
        # For a safety guard the nearest thing is the answer, stray or not.
        ranges = [5.0] * COUNT
        ranges[COUNT // 2] = 0.2
        assert cone(ranges, ANGLE_MIN, INCREMENT, 0, reducer='min') == \
            pytest.approx(0.2)

    def test_a_cone_about_behind_does_not_break_at_the_wrap(self):
        # 180 degrees sits exactly on the seam of a full-circle scan. A naive
        # comparison finds half the cone and misses the rest.
        ranges = [5.0] * COUNT
        for i in list(range(0, 4)) + list(range(COUNT - 4, COUNT)):
            ranges[i] = 0.8
        assert cone(ranges, ANGLE_MIN, INCREMENT, 180, half_deg=8) == \
            pytest.approx(0.8)

    def test_a_wider_cone_sees_more(self):
        ranges = scan(5.0, d20=1.0)
        assert cone(ranges, ANGLE_MIN, INCREMENT, 0, half_deg=5) == \
            pytest.approx(5.0)
        assert cone(ranges, ANGLE_MIN, INCREMENT, 0, half_deg=25,
                    reducer='min') == pytest.approx(1.0)


class TestNearest:

    def test_finds_the_closest_in_any_direction(self):
        assert nearest(scan(5.0, n90=0.34)) == pytest.approx(0.34)

    def test_an_empty_scan_is_infinitely_far_not_touching(self):
        # inf, so a guard comparing against a distance cannot read 'saw
        # nothing' as 'something is touching me' and refuse to move.
        assert nearest([float('nan')] * COUNT) == INF

    def test_ignores_beams_outside_the_sensor_limits(self):
        ranges = [5.0] * COUNT
        ranges[0] = 0.05                          # below the 0.10 m minimum
        assert nearest(ranges, range_min=0.10) == pytest.approx(5.0)


class TestOdometerUse:

    def test_a_straight_run_is_usable(self):
        assert is_usable_as_odometer(math.radians(1.0))

    def test_the_run_that_defeated_the_guard_is_not(self):
        # 55 degrees of curve over 2.66 m. The cone swept onto other surfaces,
        # the distance was meaningless, and a 0.5 m guard did not fire until
        # 0.22 m.
        assert not is_usable_as_odometer(math.radians(55.0))

    def test_direction_of_the_turn_does_not_matter(self):
        assert not is_usable_as_odometer(math.radians(-55.0))

    def test_travelled_is_the_shrink_in_range(self):
        assert travelled(5.56, 2.56) == pytest.approx(3.0)

    def test_a_growing_range_reports_negative_not_progress(self):
        # Exactly what happened: 5.56 m became 6.69 m while driving forward,
        # because the cone was pointing through a doorway. Straight-line travel
        # toward a surface cannot do that, so the caller must treat a negative
        # result as a broken reference rather than as reversing.
        assert travelled(5.56, 6.69) < 0


class TestIndexFor:

    def test_round_trips_through_a_bearing(self):
        for deg in (-179, -90, -1, 0, 1, 90, 179):
            i = index_for(math.radians(deg), ANGLE_MIN, INCREMENT, COUNT)
            assert i is not None
            back = math.degrees(ANGLE_MIN + i * INCREMENT)
            assert back == pytest.approx(deg, abs=math.degrees(INCREMENT))

    def test_outside_the_scan_is_none(self):
        assert index_for(math.radians(200), ANGLE_MIN, INCREMENT, COUNT) is None
