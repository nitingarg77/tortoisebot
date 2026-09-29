"""Tests for the runaway detector. No robot, no ROS.

The two cases that matter are drawn from things that actually happened to this
robot, and they pull in opposite directions: a scan-matching correction must
not trip it, and a real runaway must.
"""

import pytest

from tortoisebot_navigation.runaway import RunawayDetector


def feed(det, points):
    """Feed (t, x, y) samples, returning the sample index that tripped it."""
    for i, (t, x, y) in enumerate(points):
        if det.update(t, x, y):
            return i
    return None


def straight_line(speed, duration, rate=10.0, start=0.0):
    """Samples for a robot moving in a straight line at a constant speed."""
    n = int(duration * rate)
    return [(start + i / rate, speed * i / rate, 0.0) for i in range(n + 1)]


class TestWarmUp:

    def test_no_verdict_before_half_a_window(self):
        det = RunawayDetector(max_speed=0.5, window=2.0)
        # 0.9 s of samples at an absurd 5 m/s must still not trip it: with too
        # little history, jitter alone reads as a large speed.
        assert feed(det, straight_line(5.0, 0.9)) is None
        assert not det.tripped
        assert det.speed() is None

    def test_a_single_sample_is_not_enough(self):
        det = RunawayDetector()
        det.update(0.0, 0.0, 0.0)
        assert not det.warm
        assert det.speed() is None

    def test_becomes_warm_at_half_a_window(self):
        det = RunawayDetector(max_speed=99.0, window=2.0)
        feed(det, straight_line(0.01, 0.9))
        assert not det.warm
        feed(det, straight_line(0.01, 1.2, start=0.0))
        assert det.warm


class TestTripping:

    def test_a_real_runaway_trips(self):
        # The shortest genuine runaway seen: about 2 m in 2 s, so ~1 m/s.
        det = RunawayDetector(max_speed=0.5, window=2.0)
        assert feed(det, straight_line(1.0, 2.0)) is not None
        assert det.tripped
        assert det.trip_speed > 0.5

    def test_a_scan_matching_correction_does_not_trip(self):
        # 2026-09-25: Cartographer snapped the pose 0.33 m while the robot
        # turned in place. Over 0.5 s that is 0.63 m/s and looks like a
        # runaway; across the 2 s window it averages 0.17 m/s and is not.
        det = RunawayDetector(max_speed=0.5, window=2.0)
        samples = [(i / 10.0, 0.0, 0.0) for i in range(11)]      # 1 s still
        samples += [(1.1, 0.33, 0.0)]                            # the snap
        samples += [(1.1 + i / 10.0, 0.33, 0.0) for i in range(1, 10)]
        assert feed(det, samples) is None
        assert not det.tripped

    def test_normal_driving_does_not_trip(self):
        # The wheels manage about 0.20 m/s, and the limit is 2.5x that.
        det = RunawayDetector(max_speed=0.5, window=2.0)
        assert feed(det, straight_line(0.20, 10.0)) is None

    def test_the_glitch_that_recovered_on_its_own_would_be_caught(self):
        # It peaked at 0.94 m/s and spent 2 s above 0.5. The watchdog exists
        # because that one was only caught by a human watching.
        det = RunawayDetector(max_speed=0.5, window=2.0)
        assert feed(det, straight_line(0.94, 2.0)) is not None

    def test_trips_on_diagonal_motion_too(self):
        # Speed is a distance, not an x displacement.
        det = RunawayDetector(max_speed=0.5, window=2.0)
        pts = [(i / 10.0, 0.08 * i, 0.08 * i) for i in range(21)]
        assert feed(det, pts) is not None


class TestLatching:

    def test_stays_tripped(self):
        det = RunawayDetector(max_speed=0.5, window=2.0)
        feed(det, straight_line(1.0, 2.0))
        assert det.tripped
        # A robot that has lost track of itself does not recover by being left
        # alone, so quiet samples must not clear it.
        feed(det, straight_line(0.0, 5.0, start=100.0))
        assert det.tripped

    def test_update_reports_the_trip_once_only(self):
        det = RunawayDetector(max_speed=0.5, window=2.0)
        pts = straight_line(1.0, 3.0)
        trips = [det.update(t, x, y) for t, x, y in pts]
        assert sum(trips) == 1

    def test_reset_clears_it(self):
        det = RunawayDetector(max_speed=0.5, window=2.0)
        feed(det, straight_line(1.0, 2.0))
        det.reset()
        assert not det.tripped
        assert det.trip_speed is None
        assert det.speed() is None


class TestSampleHandling:

    def test_repeated_transforms_are_ignored(self):
        # TF hands out the same transform repeatedly between updates. Counting
        # it again would shrink the window for no reason.
        det = RunawayDetector()
        det.update(1.0, 0.0, 0.0)
        assert det.update(1.0, 9.0, 9.0) is False
        assert len(det.samples) == 1

    def test_out_of_order_samples_are_ignored(self):
        det = RunawayDetector()
        det.update(2.0, 0.0, 0.0)
        det.update(1.0, 5.0, 5.0)
        assert len(det.samples) == 1

    def test_old_samples_leave_the_window(self):
        det = RunawayDetector(max_speed=99.0, window=2.0)
        feed(det, straight_line(0.1, 10.0))
        span = det.samples[-1][0] - det.samples[0][0]
        assert span <= 2.0 + 1e-9

    def test_speed_is_measured_across_the_window_not_between_samples(self):
        # A single large jump inside an otherwise still window must be damped
        # by the window, which is the entire design decision.
        det = RunawayDetector(max_speed=99.0, window=2.0)
        feed(det, [(i / 10.0, 0.0, 0.0) for i in range(20)])
        det.update(2.0, 1.0, 0.0)
        assert det.speed() == pytest.approx(1.0 / 2.0, abs=0.1)
